**Install dependencies**
Please install [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), [Armadillo](https://www.uio.no/studier/emner/matnat/fys/FYS4411/v13/guides/installing-armadillo/) and [GNU Science Library (GSL)](https://www.gnu.org/software/gsl/) before you continue.

**Building the simulator**

Clone the following packages to your ROS workspace. i.e: (~/catkin_ws/src/). 

    git clone  https://gitlab.com/malintha/cf_simulator/-/tree/standalone
    git clone  https://gitlab.com/malintha/geo_controller/
    git clone  https://gitlab.com/malintha/simulator_utils
    
Use catkin build to build the packages as below.
    
    cd catkin_ws/ && source devel/setup.bash
    catkin build cf_simulator
