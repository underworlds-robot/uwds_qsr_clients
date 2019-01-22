#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Modified from severin lemaignan underworlds client example :
# see : https://github.com/severin-lemaignan/underworlds/blob/master/clients/spatial_relations.py

import rospy
import uuid
from pyuwds.reconfigurable_client import ReconfigurableClient
from pyuwds.types import MONITOR, MESH, Situation, Property, Changes

EPSILON = 0.005  # 5mm

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
        self.isIn = {}
        self.isOnTop = {}

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
        if len(changes.nodes_to_update) > 0:
            self.sendWorldChanges(world_name+"_allocentric_relations", header, changes)

    def monitor(self, world_name, header, invalidations):
        """
        """
        changes = Changes()
        for node_id in invalidations.node_ids_updated:
            node1 = self.worlds[world_name].scene.nodes[node_id]
            if node1.type != MESH:
                continue
            for node2 in self.worlds[world_name].scene.nodes.values():
                if node2.type != MESH:
                    continue
                bb1 = self.bb(node1)
                bb2 = self.bb(node2)
                if node1.id not in self.isin:
                    self.isIn[node1.id] = {}
                if self.isin(bb1, bb2):
                    if node2.id not in self.isIn[node1.id]:
                        sit = Situation()
                        sit.id = str(uuid())
                        sit.description = node1.name + " is in " + node2.name
                        sit.properties.append(Property("subject", node1.id))
                        sit.properties.append(Property("object", node2.id))
                        sit.start.data = header.stamp
                        sit.end.data = rospy.Time(0)
                        self.isIn[node1.id][node2.id] = sit
                        changes.situations_to_update.append(sit)
                else:
                    if node2.id in self.isIn[node1.id]:
                        sit = self.isIn[node1.id][node2.id]
                        sit.end.data = header.stamp
                        changes.situations_to_update.append(sit)
                        del self.isIn[node1.id][node2.id]

                if self.isontop(bb1, bb2):
                    if node2.id not in self.isOnTop[node1.id]:
                        sit = Situation()
                        sit.id = str(uuid())
                        sit.description = node1.name + " is on " + node2.name
                        sit.properties.append(Property("subject", node1.id))
                        sit.properties.append(Property("object", node2.id))
                        sit.start.data = header.stamp
                        sit.end.data = rospy.Time(0)
                        self.isIn[node1.id][node2.id] = sit
                        changes.situations_to_update.append(sit)
                else:
                    if node2.id in self.isOnTop[node1.id]:
                        sit = self.isOnTop[node1.id][node2.id]
                        sit.end.data = header.stamp
                        changes.situations_to_update.append(sit)
                        del self.isOnTop[node1.id][node2.id]
        return changes

    def bb(self, node):
        """
        """
        for property in node.properties:
            if property.name == "aabb":
                aabb = property.data.split(",")
                if len(aabb) == 3:
                    return int(aabb[0]), int(aabb[1]), int(aabb[3])
        raise RuntimeError("aabb not present")

    def bb_center(self, bb):
        """
        """
        x1, y1, z1 = bb[0]
        x2, y2, z2 = bb[1]

        return x1 + x2 / 2, y1 + y2 / 2, z1 + z2 / 2

    def bb_footprint(self, bb):
        """
        """
        x1, y1, z1 = bb[0]
        x2, y2, z2 = bb[1]

        return (x1, y1), (x2, y2)

    def overlap(self, rect1, rect2):
        """Overlapping rectangles overlap both horizontally & vertically
        """
        (l1, b1), (r1, t1) = rect1
        (l2, b2), (r2, t2) = rect2
        return self.range_overlap(l1, r1, l2, r2) and self.range_overlap(b1, t1, b2, t2)

    def range_overlap(self, a_min, a_max, b_min, b_max):
        """Neither range is completely greater than the other

        http://codereview.stackexchange.com/questions/31352/overlapping-rectangles
        """
        return (a_min <= b_max) and (b_min <= a_max)

    def weakly_cont(self, rect1, rect2):
        """Obj1 is weakly contained if the base of the object is surrounded
        by Obj2
        """
        (l1, b1), (r1, t1) = rect1
        (l2, b2), (r2, t2) = rect2
        return (l1 >= l2) and (b1 >= b2) and (r1 <= r2) and (t1 <= t2)

    def isabove(self, bb1, bb2, prev=False):
        """
        For obj 1 to be above obj 2:
             - the bottom of its bounding box must be higher that
               the top of obj 2's bounding box
             - the bounding box footprint of both objects must overlap
        :param bb1:
        :param bb2:
        :param prev:
        :return:
        """
        bb1_min, _ = bb1
        _, bb2_max = bb2

        x1, y1, z1 = bb1_min
        x2, y2, z2 = bb2_max

        if z1 < z2 - EPSILON:
            return False

        return self.overlap(self.bb_footprint(bb1),
                            self.bb_footprint(bb2))

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

        if z1 > z2 + EPSILON:
            return False

        if z1 < z3 - EPSILON:
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

        return z1 < z2 + EPSILON and self.isabove(bb1, bb2, prev)


if __name__ == '__main__':
    rospy.init_node("allocentric_monitor")
    monitor = AllocentricMonitor()
    rospy.spin()
