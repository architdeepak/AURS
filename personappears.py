import rospy
import random
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

def spawn_person(model_name="person", model_path="/home/archi/.gazebo/models/person_standing/model.sdf"):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    try:
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        pose = Pose()
        pose.position.x = random.uniform(0, 30) 
        pose.position.y = random.uniform(-30,0) 
        pose.position.z = 0.0 

        with open(model_path, "r") as model_file:
            model_xml = model_file.read()

        spawn_model(model_name, model_xml, "", pose, "world")
        rospy.loginfo(f"Spawned {model_name} at ({pose.position.x}, {pose.position.y})")

    except rospy.ServiceException as e:
        rospy.logerr(f"Spawn failed: {e}")

def delete_person(model_name="person"):
    rospy.wait_for_service("/gazebo/delete_model")
    try:
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model(model_name)
        rospy.loginfo(f"Deleted {model_name}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Delete failed: {e}")

def random_spawn_loop():
    rospy.init_node("random_person_spawner", anonymous=True)
    while not rospy.is_shutdown():
        spawn_person()

        #delete_person()

if __name__ == "__main__":
    try:
        random_spawn_loop()
    except rospy.ROSInterruptException:
        pass
