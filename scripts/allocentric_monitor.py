#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Modified from severin lemaignan underworlds client example :
# see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py

import rospy
import uuid
import numpy as np
import tf
from pyuwds.reconfigurable_client import ReconfigurableClient
from uwds_msgs.msg import Situation, Property, Changes
from pyuwds.types import MONITOR, MESH, FACT

EPSILON = 0.01  # 1cm

IN_CONFIDENCE = 0.75
ONTOP_CONFIDENCE = 0.95

class AllocentricMonitor(ReconfigurableClient):
    """
    """
    def __init__(self):
        """
        """
        self.isIn = {}
        self.isOnTop = {}
        ReconfigurableClient.__init__(self, "allocentric_monitor", MONITOR)

    def onReconfigure(self, worlds_names):
        """
        """
        pass

    def onSubscribeChanges(self, world_name):
        """
        """
        pass

    def onUnsubscribeChanges(self, world_name):
        """
        """
        pass

    def onChanges(self, world_name, header, invalidations):
        """
        """
        changes = self.monitor(world_name, header, invalidations)
        if len(changes.situations_to_update) > 0:
            self.sendWorldChanges("allocentric_relations", header, changes)

    def monitor(self, world_name, header, invalidations):
        """
        """
        changes = Changes()
        now = rospy.Time.now()

        for situation_id in invalidations.situation_ids_updated:
            changes.situations_to_update.append(self.worlds[world_name].timeline.situations[situation_id])

        for node1 in self.worlds[world_name].scene.nodes.values():
            if node1.type != MESH:
                continue
            for node2 in self.worlds[world_name].scene.nodes.values():
                if node1.id == node2.id:
                    continue
                if node2.type != MESH:
                    continue
                bb1 = self.aabb(node1)
                bb2 = self.aabb(node2)
                if node1.id not in self.isIn:
                    self.isIn[node1.id] = {}
                if node1.id not in self.isOnTop:
                    self.isOnTop[node1.id] = {}
                if self.isin(bb1, bb2, node2.id in self.isIn[node1.id]):
                    if node2.id not in self.isIn[node1.id]:
                        sit = Situation()
                        sit.id = str(uuid.uuid4())
                        sit.type = FACT
                        sit.description = node1.name + " is in " + node2.name
                        sit.properties.append(Property("subject", node1.id))
                        sit.properties.append(Property("object", node2.id))
                        sit.properties.append(Property("predicate", "isIn"))
                        sit.confidence = IN_CONFIDENCE
                        sit.start.data = now
                        sit.end.data = rospy.Time(0)
                        self.isIn[node1.id][node2.id] = sit
                        changes.situations_to_update.append(sit)
                else:
                    if node2.id in self.isIn[node1.id]:
                        self.isIn[node1.id][node2.id].end.data = now
                        sit = self.isIn[node1.id][node2.id]
                        changes.situations_to_update.append(sit)
                        del self.isIn[node1.id][node2.id]

                if self.isontop(bb1, bb2, node2.id in self.isOnTop[node1.id]):
                    if node2.id not in self.isOnTop[node1.id]:
                        sit = Situation()
                        sit.id = str(uuid.uuid4())
                        sit.type = FACT
                        sit.description = node1.name + " is on " + node2.name
                        sit.properties.append(Property("subject", node1.id))
                        sit.properties.append(Property("object", node2.id))
                        sit.properties.append(Property("predicate", "isOnTop"))
                        sit.confidence = ONTOP_CONFIDENCE
                        sit.start.data = now
                        sit.end.data = rospy.Time(0)
                        self.isOnTop[node1.id][node2.id] = sit
                        changes.situations_to_update.append(sit)
                else:
                    if node2.id in self.isOnTop[node1.id]:
                        self.isOnTop[node1.id][node2.id].end.data = now
                        sit = self.isOnTop[node1.id][node2.id]
                        changes.situations_to_update.append(sit)
                        del self.isOnTop[node1.id][node2.id]

        return changes

    def aabb(self, node):
        """
        Compute Aligned Axis Bounding Box from Oriented Axis Bounding Box by transforming the corners of the oabb by the node pose
        """
        for property in node.properties:
            if property.name == "aabb":
                aabb = property.data.split(",")
                if len(aabb) == 3:
                    t = [node.position.pose.position.x, node.position.pose.position.y, node.position.pose.position.z]
                    q = [node.position.pose.orientation.x, node.position.pose.orientation.y, node.position.pose.orientation.z, node.position.pose.orientation.w]
                    trans = tf.transformations.translation_matrix(t)
                    rot = tf.transformations.quaternion_matrix(q)
                    transform = tf.transformations.concatenate_matrices(trans, rot)
                    v = []
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([ float(aabb[0])/2,  float(aabb[1])/2, float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([-float(aabb[0])/2,  float(aabb[1])/2, float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([ float(aabb[0])/2, -float(aabb[1])/2, float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([-float(aabb[0])/2, -float(aabb[1])/2, float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([ float(aabb[0])/2,  float(aabb[1])/2, -float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([-float(aabb[0])/2,  float(aabb[1])/2, -float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([ float(aabb[0])/2, -float(aabb[1])/2, -float(aabb[2])/2]))))
                    v.append(tf.transformations.translation_from_matrix(np.dot(transform, tf.transformations.translation_matrix([-float(aabb[0])/2, -float(aabb[1])/2, -float(aabb[2])/2]))))
                    bb_min = [1e10, 1e10, 1e10]
                    bb_max = [-1e10, -1e10, -1e10]
                    for vertex in v:
                        bb_min = np.minimum(bb_min, vertex)
                        bb_max = np.maximum(bb_max, vertex)
                    return bb_min, bb_max
        raise RuntimeError("aabb not present")

    def bb_footprint(self, bb):
        """
        """
        x1, y1, z1 = bb[0]
        x2, y2, z2 = bb[1]

        return (x1, y1), (x2, y2)

    def overlap(self, rect1, rect2, prev=False):
        """Overlapping rectangles overlap both horizontally & vertically
        """
        (l1, b1), (r1, t1) = rect1
        (l2, b2), (r2, t2) = rect2
        return self.range_overlap(l1, r1, l2, r2) and self.range_overlap(b1, t1, b2, t2)

    def range_overlap(self, a_min, a_max, b_min, b_max, prev=False):
        """Neither range is completely greater than the other

        http://codereview.stackexchange.com/questions/31352/overlapping-rectangles
        """
        if prev is False:
            return (a_min <= b_max - EPSILON) and (b_min <= a_max + EPSILON)
        else:
            return (a_min <= b_max + EPSILON) and (b_min <= a_max - EPSILON)

    def weakly_cont(self, rect1, rect2, prev=False):
        """Obj1 is weakly contained if the base of the object is surrounded
        by Obj2
        """
        (l1, b1), (r1, t1) = rect1
        (l2, b2), (r2, t2) = rect2
        if prev is False:
            return (l1 >= l2 - EPSILON) and (b1 >= b2 - EPSILON) and (r1 <= r2 + EPSILON) and (t1 <= t2 - EPSILON)
        else:
            return (l1 >= l2 - EPSILON) and (b1 >= b2 - EPSILON) and (r1 <= r2 - EPSILON) and (t1 <= t2 + EPSILON)

    def isabove(self, bb1, bb2, prev=False):
        """ For obj 1 to be above obj 2:
             - the bottom of its bounding box must be higher that
               the top of obj 2's bounding box
             - the bounding box footprint of both objects must overlap
        """

        bb1_min, _ = bb1
        _, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max

        if z1 < z2 - EPSILON:
            return False

        return self.overlap(self.bb_footprint(bb1), self.bb_footprint(bb2))

    def isin(self, bb1, bb2, prev=False):
        """ Returns True if bb1 is in bb2.

        To be 'in' bb1 is weakly contained by bb2 and the bottom of bb1 is lower
        than the top of bb2 and higher than the bottom of bb2.
        """
        bb1_min, _ = bb1
        bb2_min, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max
        x3, y3, z3 = bb2_min

        if z1 > z2 - EPSILON:
            return False

        if z1 < z3 + 2*EPSILON:
            return False

        return self.weakly_cont(self.bb_footprint(bb1), self.bb_footprint(bb2))

    def isontop(self, bb1, bb2, prev=False):
        """ For obj 1 to be on top of obj 2:
             - obj1 must be above obj 2
             - the bottom of obj 1 must be close to the top of obj 2
        """
        bb1_min, _ = bb1
        _, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max

        return z1 < z2 + EPSILON and self.isabove(bb1, bb2)


if __name__ == '__main__':
    rospy.init_node("allocentric_monitor")
    monitor = AllocentricMonitor()
    rospy.spin()
