
      for (  std::vector<hueblob::Blob>::iterator iter= blobs.blobs.begin();
             iter != blobs.blobs.end(); iter++ )
        {
          if (!leftImage_ || !rightImage_)
            break;
          int x =  (*iter).boundingbox_2d[0];
          int y =  (*iter).boundingbox_2d[1];
          int width =  (*iter).boundingbox_2d[2];
          int height =  (*iter).boundingbox_2d[3];
          cv::Point p1(x, y);
          cv::Point p2(x + width, y + height);
          cv::Point pc(x, y + std::max(16, height+8));
          const cv::Scalar color = CV_RGB(255,0,0);
          ROS_DEBUG_STREAM("Drawing rect " << x << " " << " " << y
                           << " " << width << " " << height);
          cv::rectangle(img, p1, p2, color, 1);
          stringstream ss (stringstream::in | stringstream::out);
	  cv::putText(img, (*iter).name, p1, CV_FONT_HERSHEY_SIMPLEX,
		      0.5, color);
          // boost::format fmter("[%3.3f %3.3f %3.3f %1.2f]");
          // (fmter % (*iter).cloud_centroid.transform.translation.x
          //  %  (*iter).cloud_centroid.transform.translation.y
          //  %  (*iter).cloud_centroid.transform.translation.z
          //  %  (*iter).depth_density
          //  );
          // cv::putText(img, fmter.str(), p1, CV_FONT_HERSHEY_SIMPLEX, 0.5, color);

          // boost::format fmter2("[%3.3f %3.3f %3.3f]");
          // (fmter2 % (*iter).position.transform.translation.x
          //  %  (*iter).position.transform.translation.y
          //  %  (*iter).position.transform.translation.z
          //  );
          // cv::putText(img, fmter2.str(), pc, CV_FONT_HERSHEY_SIMPLEX, 0.5, color);
        }

      if (leftImage_){
        cv_bridge::CvImage brd_im;
        brd_im.image = img;
        brd_im.header = leftImage_->header;
        brd_im.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        tracked_left_pub_.publish(brd_im.toImageMsg());
      }
      else
        ROS_WARN_THROTTLE(20,"leftImage_ is not received");
