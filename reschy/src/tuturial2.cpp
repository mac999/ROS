namespace enc = sensor_msgs::image_encodings;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    if (enc::isColor(msg->encoding))
      cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    else
      cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Process cv_ptr->image using OpenCV
}