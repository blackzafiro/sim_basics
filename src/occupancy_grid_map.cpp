// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
// %EndTag(INCLUDES)%


/// El eje X es rojo.
/// El eje Y es verde.
/// El eje Z apunta hacia arriba y el marcador es azul.

const int WIDTH = 24;      /// A lo largo del eje rojo x
const int HEIGHT = 31;     /// A lo largo del eje verde

void fillRectangle(char* data, int i1, int j1, int i2, int j2, int value)
{
  for(int i = i1; i <= i2; i++)
  {
    for(int j = j1; j <= j2; j++)
    {
      data[i*WIDTH+j] = value;
    }
  }
}

// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_map");
  ros::NodeHandle n;
  ros::Rate r(1);
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher marker_pub = n.advertise<nav_msgs::OccupancyGrid>("visualization_marker", 1);
// %EndTag(INIT)%

// %Tag(MAP_INIT)%
  while (ros::ok())
  {
    nav_msgs::OccupancyGrid map;

    // http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
    map.header.frame_id = "/odom";
    map.header.stamp = ros::Time::now();   // No caduca

    map.info.resolution = 0.3;             // [m/cell]
    map.info.width = WIDTH;                // [cells]
    map.info.height = HEIGHT;              // [cells]
    map.info.origin.position.x = 0;
    map.info.origin.position.y = 0;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    //int8[] &_data = &map.data
    int size = WIDTH * HEIGHT;
    char* data = new char[size];
    for(int i = 0; i < size; i++) {
      data[i] = 0;
    }

    data[0] = 50;                            // El origen está en la esquina inferior izquierda.
    fillRectangle(data, 0, 1, 0, WIDTH-1, 100);   // Renglón 0. Las columnas van de 0 a WIDTH-1.  Los renglones corren sobre el eje Y.
    fillRectangle(data, 1, 0, HEIGHT-1, 0, 100);  // Columna 0. Los renglones va de 0 a HEIGHT-1.  Las columnas corren sobre el eje X.
    
    //fillRectangle(data, 0, 0, 2, 2);
    //data[5*WIDTH+3] = 100;
    //data[5*WIDTH+2] = -1;
    map.data = std::vector<int8_t>(data, data + size);
  
// %EndTag(MAP_INIT)%

    marker_pub.publish(map);

// %Tag(SLEEP_END)%
    r.sleep();
  }
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
