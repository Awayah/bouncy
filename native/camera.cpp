
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <map>

#include "camera.h"

// Define subpixSampleSafe function
int subpixSampleSafe(const cv::Mat &pSrc, const cv::Point2f &p) {
    int x = int( floorf ( p.x ) );
    int y = int( floorf ( p.y ) );
    if ( x < 0 || x >= pSrc.cols - 1 || y < 0 || y >= pSrc.rows - 1 ) return 127;
    int dx = int ( 256 * ( p.x - floorf ( p.x ) ) );
    int dy = int ( 256 * ( p.y - floorf ( p.y ) ) );
    unsigned char* i = ( unsigned char* ) ( ( pSrc.data + y * pSrc.step ) + x );
    int a = i[ 0 ] + ( ( dx * ( i[ 1 ] - i[ 0 ] ) ) >> 8 );
    i += pSrc.step;
    int b = i[ 0 ] + ( ( dx * ( i[ 1 ] - i[ 0 ] ) ) >> 8 );
    return a + ( ( dy * ( b - a) ) >> 8 );
}

Camera::Camera(int camera) : fps(30), flip_lr(false), flip_ud(false), threshold(127){

    capture.open(camera);

    if (!capture.isOpened()) {
        capture.release();
        throw std::runtime_error("Unable to open camera");
    }

    width = (int) capture.get(CAP_PROP_FRAME_WIDTH);
    height = (int) capture.get(CAP_PROP_FRAME_HEIGHT);


    std::cout << "Camera ready (" << width << "x" << height << ")" << std::endl;

    worker = std::thread(&Camera::loop, this);

}

Camera::~Camera() {

    {

        std::cout << "Closing camera" << std::endl;

        std::lock_guard<std::recursive_mutex> lock(guard);
        capture.release();

    }

    worker.join();

}

Mat Camera::getFrame() {

    std::lock_guard<std::recursive_mutex> lock(guard);

	return frame;
}


unsigned long Camera::getFrameNumber() {

    std::lock_guard<std::recursive_mutex> lock(guard);

    return counter;

}

Mat Camera::getFrameIfNewer(unsigned long& current) {

    std::lock_guard<std::recursive_mutex> lock(guard);

    if (current == counter) return Mat();

    current = counter;

    return frame;

}

int Camera::getWidth() {

    return width;

}


int Camera::getHeight() {

    return height;

}

int Camera::flip(bool flip_lr, bool flip_ud) {

    this->flip_lr = flip_lr;
    this->flip_ud = flip_ud;

    return 1;

}

int Camera::threshold_value(int threshold) {

    this->threshold = threshold;
    return threshold;

}

void Camera::loop() {

    while (true) {

        auto start = std::chrono::high_resolution_clock::now();

        {

            std::lock_guard<std::recursive_mutex> lock(guard);

            capture.read(frame);

            if (frame.empty()) {
                break;
            }

            if (flip_lr || flip_ud) {
                int code = flip_lr ? (flip_ud ? -1 : 1) : 0;
                cv::flip(frame, frame, code);
            }
            
            // Apply thresholding
            this->computeThreshold(&frame);

            this->processFrame(frame);

            
            counter++;

        }

        auto end = std::chrono::high_resolution_clock::now();

        auto used = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

		auto remaining = std::chrono::milliseconds(std::max(1l, (long)((1000.0 / fps) - used)));

        std::this_thread::sleep_for(remaining);

    }
}



static std::map<int, std::weak_ptr<Camera> > cameras;

static int default_camera = 0;

SharedCamera camera_open(int id) {

    if (id < 0) id = default_camera;

    std::cout << "Query camera " << id << std::endl;

    if (cameras.find(id) == cameras.end()) {

        try {

            SharedCamera camera = std::make_shared<Camera>(id);

            cameras[id] = camera;

            std::cout << "Ready camera " << id << std::endl;

            return camera;

        } catch (const std::runtime_error& e) {
            return std::shared_ptr<Camera>();
        }

    } else {

        return cameras[id].lock();

    }

}

void camera_delete_all() {

    cameras.clear();

}

void* camera_create(int id) {

    SharedCamera camera = camera_open(id);

    return new SharedCamera(camera);

}

void camera_delete(void *obj) {

    if (!obj) return;

    delete (SharedCamera*) obj;

}

int camera_get_image(void *obj, uint8_t* buffer, unsigned long* newer) {

    SharedCamera user_data = *((SharedCamera*) obj);

    Mat frame;

    if (newer)
        frame = user_data->getFrameIfNewer(*newer);
    else {
        frame = user_data->getFrame();
    }

    if (frame.empty()) {
        return 0;
    }

    Mat wrapper(user_data->getHeight(), user_data->getWidth(), CV_8UC3, buffer, std::max(user_data->getHeight(), user_data->getWidth()) * 3);
    cvtColor(frame, wrapper, COLOR_BGR2RGB);

	return 1;
}

int camera_get_width(void *obj) {

    SharedCamera user_data = *((SharedCamera*) obj);

    return user_data->getWidth();
}


int camera_get_height(void *obj) {

    SharedCamera user_data = *((SharedCamera*) obj);

    return user_data->getHeight();
}

void camera_set_default(int id) {
    default_camera = id;
}

void camera_flip(void *obj, int flip_lr, int flip_ud) {

    SharedCamera user_data = *((SharedCamera*) obj);

    user_data->flip(flip_lr, flip_ud);
}

void camera_threshold(void *obj, int threshold) {

    SharedCamera user_data = *((SharedCamera*) obj);

    user_data->threshold_value(threshold);
}

void Camera::computeThreshold(cv::Mat* frame_out)
{
    cv::cvtColor(*frame_out, *frame_out, COLOR_BGR2GRAY);

    if (this->threshold == 0)
    {
        cv::adaptiveThreshold(*frame_out, *frame_out, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 33, 5);
    }
    else {
        cv::threshold(*frame_out, *frame_out, this->threshold, 255, THRESH_BINARY);
    }
}


void Camera::processFrame(cv::Mat& frame) {

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (frame.channels() == 1) {
    cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);}

    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true) * 0.02, true);

         // Skip polygons with more or less than 4 corners
        if (approx.size() != 4) {
            continue;
        }

        // Compute bounding box
        cv::Rect boundingBox = cv::boundingRect(approx);

        // Skip too small bounding boxes
        if (boundingBox.width < 20 || boundingBox.height < 20) { // Experiment with these values
            continue;
        }

        // Draw contours and bounding box
        cv::drawContours(frame, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(0, 0, 255), 3);
        // Draw bounding box with red lines using cv::polylines
        std::vector<cv::Point> points = {boundingBox.tl(), cv::Point(boundingBox.br().x, boundingBox.tl().y), boundingBox.br(), cv::Point(boundingBox.tl().x, boundingBox.br().y)};
        cv::polylines(frame, points, true, cv::Scalar(255, 0, 0), 2);

        // Subdivide each edge into 7 parts of equal length
        for (int i = 0; i < 4; i++) {
            cv::Point p1 = points[i];
            cv::Point p2 = points[(i+1)%4];
            std::vector<cv::Point> subdividedPoints;

            for (int j = 0; j <= 7; j++) {
                int x = p1.x + j * (p2.x - p1.x) / 7;
                int y = p1.y + j * (p2.y - p1.y) / 7;
                subdividedPoints.push_back(cv::Point(x, y));
            }

            // Draw a small circle around each of the dividing points
            for (const cv::Point& p : subdividedPoints) {
                cv::circle(frame, p, 3, cv::Scalar(0, 255, 0), -1); // Draw circles in green
            }
        }

        
    }
}