from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.motion_generation.lula import RmpFlow
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import cuboid
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
import numpy as np
import os
import argparse


#### Libraries to set-up the demo-factory (Added by SWP) ####
import numpy as np
from omni.isaac.core.utils.prims import create_prim, delete_prim,set_prim_property, get_prim_property
from omni.isaac.core.utils.rotations import euler_angles_to_quat #To transform euler angle to quat
import math

from omni.isaac.core.utils.prims import set_prim_property, get_prim_property
from pxr import Gf


# Add libraries for Contact report
from omni.physx import get_physx_interface, get_physx_simulation_interface
from pxr import UsdGeom, Sdf, Gf, Vt, PhysicsSchemaTools
import contact_report as cr

#### Added by RheeKihun ###
import omni.kit.pipapi

omni.kit.pipapi.install(
    package="bayesian-optimization",
    version="1.4.2",
    module="bayes_opt", # sometimes module is different from package name, module is used for import check
    ignore_import_check=False,
    ignore_cache=False,
    use_online_index=True,
    surpress_output=False,
    extra_args=[]
)

from bayes_opt import BayesianOptimization

#########################################################################
### TODO: Fill in tutorial directory with absolute path to this file  ###
#########################################################################
INDYRP2_DIRECTORY = "/home/user/indyrp2_description/"
USD_PATH = "omniverse://localhost/Projects/Smart_factory/CDE/linear_stage"

rmp_config_dir = os.path.join(INDYRP2_DIRECTORY,"rmpflow")

parser = argparse.ArgumentParser()
parser.add_argument("--urdf_path",type=str,default="Indyrp2.urdf")
parser.add_argument("--rmpflow_config_path",type=str,default="indyrp2_rmpflow_common.yaml")
parser.add_argument("--end_effector_frame_name",type=str,default="TCP")
args = parser.parse_args()


# open_stage(usd_path=os.path.join(INDYRP2_DIRECTORY,"Indyrp2_slide.usd"))
open_stage(usd_path=os.path.join(USD_PATH,"room114_final_ver4.usd"))

my_world = World(stage_units_in_meters=1.0)

# robot = my_world.scene.add(Robot(prim_path="/World/Indyrp2_stage/Indyrp2", name="indyrp2", position = np.array([-0.6336275,0.5555562,1.0701538])))
robot = my_world.scene.add(Robot(prim_path="/World/Indyrp2", name="indyrp2"))

# robot = my_world.scene.get_object("/World/Indyrp2")

#Initialize an RmpFlow object
rmpflow = RmpFlow(
    robot_description_path = os.path.join(rmp_config_dir,"robot_description.yaml"),
    urdf_path = os.path.join(INDYRP2_DIRECTORY,args.urdf_path),
    rmpflow_config_path = os.path.join(rmp_config_dir,args.rmpflow_config_path),
    end_effector_frame_name = args.end_effector_frame_name, #This frame name must be present in the URDF
    maximum_substep_size = .0034
)

#Uncomment this line to visualize the collision spheres in the robot_description YAML file
#rmpflow.visualize_collision_spheres()

physics_dt = 1/60.
articulation_rmpflow = ArticulationMotionPolicy(robot,rmpflow,physics_dt)

articulation_controller = robot.get_articulation_controller()

#Make a target to follow
target_cube = cuboid.VisualCuboid("/World/target",position = np.array([-0.5,0,0.5]), orientation = np.array([0,-1,0,0]) ,color=np.array([0,0,0]),size = .01)

#Create devices on a random position (Added by SWP)
init_overlap_flag = 0 #check whether the devices overlap occurs or not
robot_collision_flag = 0 #check whether the robot collides devices or not

# Set the delta translation of objects (you can remove)
delta_stage = 0.005
delta_target = 0.0002

my_world.reset()

i = 0
create_prim(    
            prim_path="/World/DP_re",
            usd_path= USD_PATH + "/3DP_re.usd",
            position = np.array([0.60,-0.60,-0.15]),
            orientation = np.array(euler_angles_to_quat([0,0,0])),
        )

flag_1 = True
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():

        # Check whether the overlap occurs at the initial creation of devices 
        if init_overlap_flag == 1:
            init_overlap_flag = 0
            my_world.reset()

        
        # Call the current position(x,y,z) and orientation, and not recomment to modify the orientation (hard to control...)
        dp1_pos =  get_prim_property(prim_path = "/World/DP_re", property_name =  "xformOp:translate")
        target_pos =  get_prim_property(prim_path = "/World/Indyrp2/Target", property_name =  "xformOp:translate")
        tcp_pos =  get_prim_property(prim_path = "/World/Indyrp2/TCP", property_name =  "xformOp:translate")
        target_ori =  get_prim_property(prim_path = "/World/Indyrp2/Target", property_name =  "xformOp:orient")
        cur_prismatic = get_prim_property(prim_path = "/World/stage_base/PrismaticJoint", property_name =  "drive:linear:physics:targetPosition")
        
        a = target_pos - tcp_pos

        print(target_pos)

        if my_world.current_time_step_index == 0:
            my_world.reset()
        


        #########################################################################################
        ########## Test to move the linear stage and manipulation (you can remove) ##############
        #########################################################################################
        i += 1
        # Move the linear stage
        set_prim_property(prim_path = "/World/stage_base/PrismaticJoint",
                            property_name =  "drive:linear:physics:targetPosition",
                            property_value = dp1_pos[1])
                        #   property_value = cur_prismatic-delta_stage)
        if dp1_pos[1]==cur_prismatic and flag_1 == True:
            flag_1 = False
            # Move the target position and the end-effector of the manipulation follows the target position
            set_prim_property(prim_path = "/World/Indyrp2/Target",
                                property_name =  "xformOp:translate",
                            property_value = Gf.Vec3d(cur_prismatic,dp1_pos[1],dp1_pos[2]))
                            #   property_value = target_pos - Gf.Vec3d(0,dp1_pos[1],0) )
                                # property_value = target_pos - Gf.Vec3d(delta_target,delta_stage,0) )


        # if i < 380:
        #     # Move the linear stage
        #     set_prim_property(prim_path = "/World/stage_base/PrismaticJoint",
        #                       property_name =  "drive:linear:physics:targetPosition",
        #                       property_value = dp1_pos[1])
        #                     #   property_value = cur_prismatic-delta_stage)
            
        #     # Move the target position and the end-effector of the manipulation follows the target position
        #     set_prim_property(prim_path = "/World/Indyrp2/Target",
        #                       property_name =  "xformOp:translate",
        #                     #   property_value = dp1_pos)
        #                     #   property_value = target_pos - Gf.Vec3d(0,dp1_pos[1],0) )
        #                       property_value = target_pos - Gf.Vec3d(delta_target,delta_stage,0) )

        # if i > 380:
        #     # Move the linear stage
        #     set_prim_property(prim_path = "/World/stage_base/PrismaticJoint",
        #                       property_name =  "drive:linear:physics:targetPosition",
        #                       property_value = cur_prismatic+delta_stage)
            
        #     # Move the target position and the end-effector of the manipulation follows the target position
        #     set_prim_property(prim_path = "/World/Indyrp2/Target",
        #                       property_name =  "xformOp:translate",
        #                       property_value = target_pos + Gf.Vec3d(delta_target,delta_stage,0) )
        # if i == 760:
        #     i = 0
        ###########################################################################################

        # RMPflow(= Motion generation) of manipulation part, and please modify carefully
        rmpflow.set_end_effector_target(
            target_position=target_pos - Gf.Vec3d(0,cur_prismatic,0),
            target_orientation=target_cube.get_world_pose()[1]
        )

        actions = articulation_rmpflow.get_next_articulation_action()
        articulation_controller.apply_action(actions)
         

        # Run collision detection function
        cr.contact_report()


        # Check whether the collision occurs while the manipulation moves 
        if robot_collision_flag == 1:
            robot_collision_flag = 0
            my_world.reset()

simulation_app.close()
