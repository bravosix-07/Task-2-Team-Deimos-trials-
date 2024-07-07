# Task-2-Team-Deimos-trials-
This repo is for the task 2 of software trials of Team Deimos.

In this task, we will create a Global Path Planner Plugin that uses A* search algorithm for autonomous naviagtion of the turtlebot3. 

Software prerequisites : Ros Noetic, turtlebot3 repo cloned in the /src folder of catkin workspace, map files created in task 1 to autonmously navigate turtlebot3 upon.

Blueprint :

   1. We create a global path planner plugin 
            
   2. We integrate it with ROS and turtlebot3.
            
   3. We use it to autonomously navigate on the map defined in Task-1 using RViz.


Here are the steps to follow :


   1. Create a package named global_planner in our catkin workspace

          cd ~/catkin_ws
      
          catkin_create_pkg my_global_planner roscpp costmap_2d base_local_planner nav_core pluginlib


   2. Creating a header file and a /.cpp file for our planner that uses A* search algorithm.
      
       Locate to src folder in the newly created package and create a file named "global_planner.h" with the following content :

          #include <string.h>
          #include <ros/ros.h>
          #include <geometry_msgs/PoseStamped.h>
          #include <tf/tf.h>
          #include <set>

          /** for global path planner interface */
          #include <costmap_2d/costmap_2d_ros.h>
          #include <costmap_2d/costmap_2d.h>       
          #include <nav_core/base_global_planner.h>
          #include <geometry_msgs/PoseStamped.h>


          using namespace std;
          using std::string;

          #ifndef GLOBAL_PLANNER_CPP
          #define GLOBAL_PLANNER_CPP

          struct GridSquare
          {
                int currentGridSquare;
                float fCost;
          };

          namespace global_planner
          {

           class GlobalPlanner : public nav_core::BaseGlobalPlanner
             {
                   public:

                   ros::NodeHandle ROSNodeHandle;
                   float originX;
                   float originY;
                   float resolution;
                   costmap_2d::Costmap2DROS *costmap_ros_;
                   costmap_2d::Costmap2D *costmap_;
                   bool initialized_;
                   int width;
                   int height;
                   GlobalPlanner(ros::NodeHandle &); //this constructor is may be not needed
                   GlobalPlanner();
                   GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  
            /** overriden methods from interface nav_core::BaseGlobalPlanner **/
            void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

            /** Helper methods**/
            void convertToMapCoordinates(float &x, float &y);
            int getGridSquareIndex(float x, float y);
            void getGridSquareCoordinates(int index, float &x, float &y);
            bool isCoordinateInBounds(float x, float y);
            vector<int> runAStarOnGrid(int startGridSquare, int goalGridSquare);
            vector<int> findPath(int startGridSquare, int goalGridSquare, float g_score[]);
            vector<int> constructPath(int startGridSquare, int goalGridSquare, float g_score[]);
            void addNeighborGridSquareToOpenList(multiset<GridSquare> &OPL, int neighborGridSquare, int goalGridSquare, float g_score[]);
            vector<int> findFreeNeighborGridSquare(int gridSquareIndex);
            bool isStartAndGoalValid(int startGridSquare, int goalGridSquare);
            float getMoveCost(int gridSquareIndex1, int gridSquareIndex2);
            float getMoveCost(int i1, int j1, int i2, int j2);
            float calculateHScore(int gridSquareIndex, int goalGridSquare);
            int calculateGridSquareIndex(int i, int j);
            int getGridSquareRowIndex(int index);
            int getGridSquareColIndex(int index);
            bool isFree(int gridSquareIndex); 
            bool isFree(int i, int j);
  
               };
            };

      


   4. Now, we create a "global_planner.cpp" file that will contain our algorithm .

      In that file, copy-paste the code given in the src/global_planner/src/global_planner.cpp given in the repo.

      OR 

      Just download the "global_planner.cpp" from the repo and place in the src/global_planner/src/ directory

      


   5. We will now integrate our package with ROS as a plugin :
 
       Open the CMAkeLists.txt file in the global_planner package and add this line :
      
          add_library(global_planner_lib src/global_planner.cpp)

       Now for plugin registeration, create a file named "global_planner_plugin.xml" in the "global_planner" package :

          <library path="lib/libglobal_planner_lib">
             <class name="global_planner/GlobalPlanner" type="global_planner::GlobalPlanner" base_class_type="nav_core::BaseGlobalPlanner">
                    <description>This is a global planner plugin by Team Deimos selection.</description>
              </class>
          </library>


      Now, open package.xml in the same directory make sure to add these lines :

            <export>
               <nav_core plugin="${prefix}/global_planner_plugin.xml" />
            </export>

      Compile your package usng "catkin_make" command.

      Check the successful creation of the plugin using this command :


            rospack plugins --attrib=plugin nav_core




  6. Configuring files for using the plugin for path planning in the autonomous navigation 

       Open the turtlebot3_navigation folder and go to move_base.launch file in the /launch folder and add this line :

            <param name="base_global_planner" value="global_planner/GlobalPlanner" />
       
       Files are also provided in the repo, you can just copy and paste
       
       Make sure to properly make changes to .yaml files in the /param folder or just simply use or copy-paste from the files given in this repo.

       After making sure that the files in "turtlebot3/turtlebot3_navigation/param" and "turtlebot3/turtlebot3_navigation/launch" are same as the ones provided, then proceed to the final step.





   7. Running the pre-defined map of task-1 with our custom global planner as a pluging for path planning

       Make sure to place "hospital_map.pgm" and "hospital_map.yaml" on your /HOME  (They are provided in the repo, download them)

       Run the following commands to launch gazebo world and RViz with turtlebot3 :


            export TURTLEBOT3_MODEL=burger
            roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
            export TURTLEBOT3_MODEL=burger
            roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/hospital_map.yaml
      

       Now you use the 2D pose estimate and 2D Nav goal in RViz to autonomously navigate your turtlebot3 in the hospital map.          

         
      
   

      

            
