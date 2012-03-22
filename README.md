# hueblob
3D color tracking

Quickstart:

  *  Install deps and compile hueblob by usual ros commands.
  *  Launch the default launch file:

         roslaunch hueblob track2-nodelet.launch

  *  This launch file launch the 3 default node(let)s, a tracker 2D and a
     projector tracked object into cartesian coordinate and a GUI.

     Theses node(let)s assume image topic /wide/left/image_rect_color is
     published and track the object declared in config.launch. To view and
     modify this object, use the GUI.

     3D position of the blob and the depth density of this blob is published
     in:

         /wide/blobs/BLOB_NAME/transform
         /wide/blobs/BLOB_NAME/density

     The default blob name is rose.
