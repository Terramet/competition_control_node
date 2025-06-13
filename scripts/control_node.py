#!/usr/bin/env python3

import rospy
from competition_control_node.srv import WatchFallDetection, WatchFallDetectionResponse
from competition_control_node.srv import VisionFallDetection, VisionFallDetectionResponse
from competition_control_node.srv import SetAssessmentState, SetAssessmentStateResponse
from competition_state_service.srv import SetState

ALLOWED_STATES = {"Idle", "Search", "Social", "Assess", "Help"}

class FallControlNode:
    def __init__(self):
        rospy.init_node('competition_control_node')

        # Set up service clients
        rospy.wait_for_service('/competition_state/set_state')
        self.set_state_client = rospy.ServiceProxy('/competition_state/set_state', SetState)

        # Create services
        rospy.Service('/fall_control/watch_detection', WatchFallDetection, self.handle_watch_detection)
        rospy.Service('/fall_control/vision_detection', VisionFallDetection, self.handle_vision_detection)
        rospy.Service('/fall_control/assessment_decision', SetAssessmentState, self.handle_assessment_state)

        rospy.loginfo("competition_control_node is ready to handle fall detection and assessment.")
        rospy.spin()

    def handle_watch_detection(self, req):
        if req.detected:
            rospy.loginfo(f"[Control] Watch fall detected at {req.timestamp}")
            self.set_state("Search")
            return WatchFallDetectionResponse(success=True, message="Fall detected")
        else:
            return WatchFallDetectionResponse(success=True, message="No fall detected")

    def handle_vision_detection(self, req):
        if req.detected:
            rospy.loginfo(f"[Control] Vision fall detected at {req.timestamp}")
            self.set_state("Assess")
            return VisionFallDetectionResponse(success=True, message="Fall detected")
        else:
            return VisionFallDetectionResponse(success=True, message="No fall detected")

    def handle_assessment_state(self, req):
        if req.new_state not in ALLOWED_STATES:
            return SetAssessmentStateResponse(success=False, message="Invalid state provided.")
        rospy.loginfo(f"[Control] Assessment decided to change state to {req.new_state}")
        return self.set_state(req.new_state)

    def set_state(self, state):
        try:
            resp = self.set_state_client(new_state=state)
            return type('Response', (object,), {"success": resp.success, "message": resp.message})()
        except rospy.ServiceException as e:
            rospy.logerr(f"[Control] Failed to set state: {e}")
            return type('Response', (object,), {"success": False, "message": str(e)})()

if __name__ == '__main__':
    try:
        FallControlNode()
    except rospy.ROSInterruptException:
        pass

