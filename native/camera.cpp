
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

Mat calculateStripDimensions(double dx, double dy, StripDimensions& st) {
    // Norm (euclidean distance) from the direction vector is the length (derived from the Pythagoras Theorem)
    double diffLength = sqrt(dx * dx + dy * dy);

    // Length proportional to the marker size
    st.stripLength = (int)(0.8 * diffLength);

    if (st.stripLength < 5)
        st.stripLength = 5;

    // Make stripeLength odd (because of the shift in nStop), Example 6: both sides of the strip must have the same length XXXOXXX
    //st.stripeLength |= 1;
    if (st.stripLength % 2 == 0)
        st.stripLength++;

    // E.g. stripeLength = 5 --> from -2 to 2: Shift -> half top, the other half bottom
    //st.nStop = st.stripeLength >> 1;
    st.nStop = st.stripLength / 2;
    st.nStart = -st.nStop;

    Size stripeSize;

    // Sample a strip of width 3 pixels
    stripeSize.width = 3;
    stripeSize.height = st.stripLength;

    // Normalized direction vector
    st.stripeVecX.x = dx / diffLength;
    st.stripeVecX.y = dy / diffLength;

    // Normalized perpendicular direction vector (rotated 90° clockwise, rotation matrix)
    st.stripeVecY.x = st.stripeVecX.y;
    st.stripeVecY.y = -st.stripeVecX.x;

    // 8 bit unsigned char with 1 channel, gray
    return Mat(stripeSize, CV_8UC1);
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

            StripDimensions strip;

            Mat image_pixel_strip = calculateStripDimensions(p2.x - p1.x, p2.y - p1.y, strip);

            for (int j = 0; j <= 7; j++) {
                int x = p1.x + j * (p2.x - p1.x) / 7;
                int y = p1.y + j * (p2.y - p1.y) / 7;
                subdividedPoints.push_back(cv::Point(x, y));
                
                // take the points that are not on the edges

                if (j == 0 || j == 7) {
                    continue;
                }

                //COlumns: Loop over 3 pixels
                for (int m = -1; m<=1; m++) {
                    // Rows: Loop over the strip
                    for (int n = strip.nStart; n <= strip.nStop; n++) {
                        cv::Point p = cv::Point(x + m * strip.stripeVecX.x + n * strip.stripeVecY.x, y + m * strip.stripeVecX.y + n * strip.stripeVecY.y);
                        int value = subpixSampleSafe(frame, p);
                        image_pixel_strip.at<uchar>(n + strip.nStop, m + 1) = value;
                    }
                }

                // Use Sobel operator on stripe over the y direction
                cv::Mat sobel_gradient_y;
                cv::Sobel(image_pixel_strip, sobel_gradient_y, CV_16S, 0, 1);

                double max_intensity = -1;
                int max_intensity_index = 0;

                // Finding the max value
                for (int n = 0; n < strip.stripLength; n++) {
                    if (sobel_gradient_y.at<short>(n, 1) > max_intensity) {
                        max_intensity = sobel_gradient_y.at<short>(n, 1);
                        max_intensity_index = n;
                    }
                }

                double y0, y1, y2;

                // Point before and after
                unsigned int max1 = max_intensity_index - 1, max2 = max_intensity_index + 1;

                // If the index is at the border we are out of the stripe, then we will take 0
                y0 = (max_intensity_index <= 0) ? 0 : sobel_gradient_y.at<uchar>(max1, 1);
                y1 = sobel_gradient_y.at<uchar>(max_intensity_index, 1);
                // If we are going out of the array of the sobel values
                y2 = (max_intensity_index >= strip.stripLength - 3) ? 0 : sobel_gradient_y.at<uchar>(max2, 1);

                // Formula for calculating the x-coordinate of the vertex of a parabola, given 3 points with equal distances 
                // (xv means the x value of the vertex, d the distance between the points): 
                // xv = x1 + (d / 2) * (y2 - y0)/(2*y1 - y0 - y2)

                // d = 1 because of the normalization and x1 will be added later
                double pos = (y2 - y0) / (4 * y1 - 2 * y0 - 2 * y2);

                // What happens when there is no solution -> /0 or Number == other Number
                // If the found pos is not a number -> there is no solution
                if (isnan(pos)) {
                    continue;
                }

                // Exact point with subpixel accuracy
                Point2d edge_center_subpix;

                // Where is the edge (max gradient) in the picture?
                int max_index_shift = max_intensity_index - (strip.stripLength >> 1);

                // Find the original edgepoint -> Is the pixel point at the top or bottom?
                edge_center_subpix.x = x + strip.stripeVecX.x + (max_index_shift * strip.stripeVecY.x);
                edge_center_subpix.y = y + strip.stripeVecX.y + (max_index_shift * strip.stripeVecY.y);

                // Highlight the subpixel with blue color
                cv::circle(frame, edge_center_subpix, 3, CV_RGB(0, 0, 255), -1);

            }

            // Draw a small circle around each of the dividing points
            for (const cv::Point& p : subdividedPoints) {
                cv::circle(frame, p, 3, cv::Scalar(0, 255, 0), -1); // Draw circles in green
            }
        }

        
    }
}