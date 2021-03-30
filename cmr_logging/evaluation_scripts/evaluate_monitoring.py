#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from datetime import datetime, timedelta, date
import pandas as pd
import matplotlib.pyplot as plt
from tqdm import tqdm
import numpy as np
import math
import pickle
import os
import copy
from collections import Counter
from strands_navigation_msgs.msg import MonitoredNavEvent
from monitoring_msgs.msg import MonitoringArray
SUCCESS_TIME_TRESH = 60.0
SUCCESS_DIST_THRESH = 1.0

def state_to_str(state):
    if state == 0:
        return "IDLE"
    if state == 1:
        return "RUNNING"
    if state == 2:
        return "SUCCESS"
    if state == 3:
        return "FAILURE"
    return "NONE"


def change_to_failure(state_change):
    changed = (
                      state_change.prev_status.value == 0 or state_change.prev_status.value == 1) and state_change.status.value == 3
    return changed


def change_to_success(state_change):
    changed = (
                      state_change.prev_status.value == 0 or state_change.prev_status.value == 1) and state_change.status.value == 2
    return changed


def change_idle_idle(uid, state_change):
    is_uid = (uid is state_change.uid)
    changed = (
                      state_change.prev_status.value == 0 or state_change.prev_status.value == 1) and state_change.status.value == 0
    return is_uid and changed


def print_activations(dct):
    for key in dct.keys():
        print(key + ": " + str(dct[key].get_num_of_activations()) + " activations")


def print_nav_activations(dct):
    for key in dct.keys():
        print(key + ": " + dct[key].str_activations())


def get_charge_intervals(charging_node):
    """Based on a Charging BT Node and its time series, get the durations of charging and operation"""
    # First we calc the gradient to check when the "Charging" condition changed from SUCCESS to FAILURE (and back)
    slope = pd.Series(np.gradient(charging_node.ds.values), charging_node.ds.index, name='slope')
    # Timepoints of slope values != 0 indicate that the state changed
    t_start_charging = slope.index[slope.values == slope.min()].tolist()
    t_stop_charging = slope.index[slope.values == slope.max()].tolist()

    def clean_list(orig_list):
        """
        Helper to clean the list of timepoints.
        Necessary because sometimes the node switches between SUCCESS and FAILURE and SUCCESS
        So we filter out all values that are too close to each other for a typical charging process (eg. less than 10 sec apart)
        """
        t_last = None
        eps = 10
        new_list = []
        if len(orig_list) < 2:
            return orig_list

        for item in orig_list:
            if t_last is not None:
                if abs((item - t_last).total_seconds()) > eps:
                    new_list.append(item)
            else:
                new_list.append(item)
            t_last = item
        return new_list
    # Cleanup the timepoints
    t_start_charging = clean_list(t_start_charging)
    t_stop_charging = clean_list(t_stop_charging)
    # If there are not as many timepoints for the start of a charging process like for the end of it,
    # this indicates that something is wrong (The robot starts its day at the charger and ends its day there)
    if len(t_start_charging) is not len(t_stop_charging):
        # Some bags are missing some measurements, so we need to cleanup manually....
        if t_start_charging[0].date() == datetime(2021, 03, 10).date():
            # Here is a gap during midday, therefore the slope was determined incorrectly
            t_start_charging = t_start_charging[1:]
        else:
            raise Exception("Start and stop charging vectors have different length")
    # Add the timepoints for the start and end of the day
    midnight_before = t_start_charging[0].replace(hour=0, minute=0, second=0, microsecond=0)
    midnight_after = t_start_charging[0].replace(hour=23, minute=59, second=59, microsecond=999999)
    t_start_charging.insert(0, midnight_before)
    t_stop_charging.append(midnight_after)
    # Difference between the two vectors now indicates the durations for charging
    charge_durs = [tsto - tsta for (tsto, tsta) in zip(t_stop_charging, t_start_charging)]
    # charge_durs_sec = map(lambda dur: dur.total_seconds(), charge_durs)
    total_charge_dur = sum(charge_durs, timedelta())
    total_operating_dur = timedelta(days=1) - total_charge_dur
    return total_charge_dur, total_operating_dur, t_start_charging, t_stop_charging


def filename_from_date(date):
    return "data_" + date.strftime("%Y-%m-%d") + ".pickle"


class MonitoredNode(object):
    def __init__(self, instance_name, nth=1):
        """
        Class to represent a BT Node that was recorded
        Args:
            instance_name (): Name of the node in the tree
            nth (): Use the nth occurrence of this node name in the tree
        """
        self.iname = instance_name
        self.last_ts_idle = rospy.Time()
        self.ts_activation = []
        self.values = []
        self.nth = nth
        self.ds = pd.Series()

    def __eq__(self, other):
        return self.iname == other.iname

    def __add__(self, other):
        new = copy.copy(self)
        if new.ds.empty:
            new.ds = other.ds
        else:
            new.ds = new.ds.append(other.ds)
        return new

    def __iadd__(self, other):
        self.ds = self.ds.append(other.ds)
        return self

    def set_uid(self, behavior_tree_msg):
        self.uid = None
        n = 1
        for node in behavior_tree_msg.nodes:
            if node.instance_name == self.iname:
                if n == self.nth:
                    self.uid = node.uid
                    return
                else:
                    n += 1

    def write_df(self):
        """ Write the data to a pandas time series object"""
        self.ds = pd.Series(self.values, index=self.ts_activation)
        self.ds.index = pd.to_datetime(self.ds.index, unit='s').tz_localize('UTC').tz_convert('Europe/Berlin')

    def save_if_activated(self, state_change):
        """
        Based on a state change, check if this node was changed an determine if it was a change to success or failure
        Args:
            state_change (): BT msg state change to check

        Returns:

        """
        if self.uid is not state_change.uid:
            return

        to_success = change_to_success(state_change)
        to_failure = change_to_failure(state_change)
        to_idle = change_idle_idle(self.uid, state_change)
        if to_success or to_failure:
            # Save all timestamps of activation
            self.ts_activation.append(state_change.timestamp.to_time())
            if to_success:
                self.values.append(state_change.status.SUCCESS)
            else:
                self.values.append(state_change.status.FAILURE)
        elif to_idle:
            # Due to a bug in the logger for the first week of evaluation no success values were stored
            # Instead all states with SUCCESS (int value 2) were written as IDLE (int value 0)
            # To Count the changes from IDLE to SUCCESS, we therefore need to count the changes from IDLE to 'IDLE'.
            # We do not want to count the changes from SUCCESS to IDLE (or 'IDLE' to IDLE), so we check the timestamp of the last IDLE to IDLE change.
            # If it is very young, this indicates that we have a change back to IDLE
            # print("Change: " + str(datetime.utcfromtimestamp(state_change.timestamp.secs) + timedelta(microseconds=state_change.timestamp.nsecs / 1000)))
            if (state_change.timestamp - self.last_ts_idle) > rospy.Duration(0.1):
                self.last_ts_idle = state_change.timestamp
                self.ts_activation.append(state_change.timestamp.to_time())
                self.values.append(state_change.status.SUCCESS)

    def get_num_of_activations(self):
        return self.ds.size

    def get_timestamps(self):
        return self.ts_activation

    def filter_around_timepoints(self, tp_list, t_delta):
        """
        Filter (drop) values from data series, which lie around specific timepoints
        Can be used to ignore values during specific timepoints
        Args:
            tp_list (): List of timepoints around which to drop
            t_delta (): timedelta to define interval around timepoints

        Returns:

        """
        for tp in tp_list:
            self.ds = self.ds.drop(self.ds[(tp - t_delta < self.ds.index) & (self.ds.index < tp + t_delta)].index)

class NavigationRecovery(object):
    def __init__(self, name):
        """
        Class to represent a Navigation recovery (STRANDS monitored navigation)
        Args:
            name (): Name of the recovery as it is used in the Monitored Navigation framework
        """
        self.name = name
        self.num_success = 0
        self.num_fail = 0
        self.poses = list()
        self.tp_start = list()
        self.time_spent = timedelta(0)

    def __add__(self, other):
        """ Define operator to sum values of multiple objects of this type"""
        new = copy.copy(self)
        new.num_success = self.num_success + other.num_success
        new.num_fail = self.num_fail + other.num_fail
        new.poses.append(other.poses)
        new.tp_start.append(other.tp_start)
        new.time_spent += other.time_spent
        return new

    def __iadd__(self, other):
        self.num_success += other.num_success
        self.num_fail += other.num_fail
        self.poses.append(other.poses)
        self.tp_start.append(other.tp_start)
        self.time_spent += other.time_spent
        return self

    def save_if_activated(self, nav_event, next_event):
        """
        Based on a strands_navigation_msgs.msg.MonitoredNavEvent msg check if this recovery behhavior was activated
        Also check if the recovery was successful, based on the next monitoredEvent in the list
        Args:
            nav_event ():
            next_event ():

        Returns:

        """
        # First we need to check if the event msg corresponds to this class instance
        if nav_event.recover_mechanism == self.name:
            if nav_event.recover_mechanism == "backtrack" and next_event.recover_mechanism == "recover_dwa_stuck_backwards":
                return

            if next_event is not None:
                if self.was_successful(nav_event, next_event):
                    self.num_success += 1
                else:
                    self.num_fail += 1
            else:
                # If there is no next event we always consider the event as successful
                self.num_success += 1

            pose = np.array((nav_event.event_start_pose.position.x, nav_event.event_start_pose.position.y))
            self.poses.append(pose)
            self.tp_start.append(nav_event.event_start_time)
            self.time_spent += timedelta(seconds=(nav_event.event_end_time - nav_event.event_start_time).to_sec())

    def was_successful(self, nav_event, next_event):
        """Check if the event was successful"""
        # If there is another event (later event) we check if this event happened close to (euclidian dist below threshold)
        # or shortly after (time difference below threshold) this event. If yes, then we consider this recovery event as
        # unsuccessful
        pose = np.array((nav_event.event_start_pose.position.x, nav_event.event_start_pose.position.y))
        pose_next = np.array((next_event.event_start_pose.position.x, next_event.event_start_pose.position.y))
        t = nav_event.event_start_time
        t_next = next_event.event_start_time
        is_not_close = np.linalg.norm(pose - pose_next) > SUCCESS_DIST_THRESH
        is_not_short_before = (t_next - t).to_sec() > SUCCESS_TIME_TRESH
        return is_not_close and is_not_short_before

    def get_num_of_activations(self):
        return self.num_success + self.num_fail

    def str_activations(self):
        total = self.num_success + self.num_fail
        if total > 0:
            perc_success = float(self.num_success) / float(total) * 100
            perc_success_str = "{:.1f}".format(perc_success)
        else:
            perc_success_str = "/"
        return "num successful: " + str(self.num_success) \
               + ", num failure: " + str(self.num_fail) \
               + ", perc successful: " + perc_success_str + "%" \
               + ", time spent: " + str(self.time_spent)


class OdometryDistance(object):
    def __init__(self, topic='/odometry/filtered'):
        """Simple class to calculate a traveled distance based on odometry values"""
        self.init = True
        self.ox = 0
        self.oy = 0
        self.dist = 0
        self.topic = topic

    def calc_total_dist(self, bag):
        """Read values from a bagfile"""
        for _, msg, _ in bag.read_messages(topics=[self.topic]):
            self.add_value(msg.pose.pose.position)

    def add_value(self, position):
        px = position.x
        py = position.y
        if not self.init:
            self.dist += math.sqrt((px - self.ox) ** 2 + (py - self.oy) ** 2)
        self.init = False
        self.ox = px
        self.oy = py

    def get_distance(self):
        return self.dist

    def __add__(self, other):
        return self.dist + other.dist

    def __iadd__(self, other):
        self.dist += other.dist
        return self


class EvaluationData(object):
    def __init__(self):
        """
        Class to store all evaluation data for a specific time period.
        The class holds dicts of all monitored BT nodes and navigation recoveries
        Data can be given by a bagfile. It was designed in a way that every objects holds the data for one day (one bag file per day)
        The objects can then be summed up and all important measures will of the child objects will be summed up accordingly
        """
        # Dict for navigation recoveries
        self.nav_recoveries = {"recover_dwa_stuck_backwards": NavigationRecovery("recover_dwa_stuck_backwards"),
                               "sleep_and_retry": NavigationRecovery("sleep_and_retry"),
                               "backtrack": NavigationRecovery("backtrack"),
                               "recover_blocked_us_sensor": NavigationRecovery("recover_blocked_us_sensor"),
                               "nav_help": NavigationRecovery("nav_help"),
                               "recover_lost_localization": NavigationRecovery("recover_lost_localization")
                               }
        # Dict for nodes of the "Monitoring BT" This includes the nodes for localization restarts and charging checks
        self.nodes_mon = {"RotateToRelocalize": MonitoredNode("SetFirstStrike"),
                          "RestartLocalization": MonitoredNode("SetSecondStrike"),
                          "RotateToRelocalizeAfterRestart": MonitoredNode("SetThirdStrike"),
                          "CallSupervisorLocalization": MonitoredNode("SetFourthStrike"),
                          "KillMoveBase": MonitoredNode("KillROSNode", 5),
                          "KillROSNode": MonitoredNode("KillROSNode", 6),
                          "ReconfigureCharging": MonitoredNode("ReconfigureMonitoring", 1),
                          "ReconfigureOperating": MonitoredNode("ReconfigureMonitoring", 2),
                          "Charging": MonitoredNode("Charging"),
                          }
        # Dict for the nodes of the "CheckMask" BT. Can be used to check how many masks /people without mask were detected
        self.nodes_mask = {"CallNLPNoMask": MonitoredNode("CallNLPNoMask"),
                           "MaskDetected": MonitoredNode("MaskDetected"),
                           "NothingDetected": MonitoredNode("NothingDetected")
                           }
        # Dict for nodes of the "PatrolBT" Can be used to check how often the robot docked successfully and how often it failed
        self.nodes_patrol = {"Docked": MonitoredNode("WaitNode", 5),
                             "ShutdownRobot": MonitoredNode("ShutdownRobot")
                             # If the robot is shutdown, docking went wrong
                             }
        self.odom_dist = OdometryDistance()
        self.num_docks_failed = 0
        self.num_docks_successful = 0
        self.t_charging = timedelta(0)
        self.t_operating = timedelta(0)
        self.tp_start_charge = list()
        self.tp_stop_charge = list()
        self.date = datetime.today()
        self.tsl_max = None
        self.tsl_avg = None
        self.t_recovering = timedelta(0)
        self.t_standing = timedelta(0)
        self.t_eval = timedelta(0)
        self.perc_active = None

    def __add__(self, other):
        new = copy.copy(self)
        return self.add(new, other)

    def __iadd__(self, other):
        return self.add(self, other)

    def add(self, obj, other):
        """Helper to add to objects of this type"""
        obj.t_charging += other.t_charging
        obj.t_operating += other.t_operating
        obj.num_docks_successful += other.num_docks_successful
        obj.num_docks_failed += other.num_docks_failed
        obj.odom_dist += other.odom_dist
        obj.nodes_patrol = dict(Counter(obj.nodes_patrol) + Counter(other.nodes_patrol))
        obj.nodes_mon = dict(Counter(obj.nodes_mon) + Counter(other.nodes_mon))
        obj.nodes_mask = dict(Counter(obj.nodes_mask) + Counter(other.nodes_mask))
        obj.nav_recoveries = dict(Counter(obj.nav_recoveries) + Counter(other.nav_recoveries))
        obj.t_standing += other.t_standing
        obj.t_recovering += other.t_recovering
        return obj

    def set_from_bag(self, bag):
        """
        Based on a bag file, read all messages and store them in the corresponding objects
        Args:
            bag (): bagfile to read

        Returns:

        """
        self.date = datetime.fromtimestamp(bag.get_end_time()).date()
        print("Date: " + self.date.strftime("%Y-%m-%d"))
        # Calculate the total traveled distance for this bagfile based on odometry
        self.odom_dist.calc_total_dist(bag)

        # Check how many nav recoveries were executed
        self.write_recoveries_from_bag(self.nav_recoveries, bag, '/monitored_navigation/monitored_nav_event')
        print_nav_activations(self.nav_recoveries)
        self.calc_time_recovering()

        self.write_stand_time_from_bag(bag, '/monitoring')


        # Check how often the Monitoring BT Nodes were activated and if they were successful
        self.write_nodes_from_bag(self.nodes_mon, bag, '/monitoring_bt/behavior_tree_log')
        self.t_charging, self.t_operating, self.tp_start_charge, self.tp_stop_charge = get_charge_intervals(self.nodes_mon["Charging"])

        # Get the timepoints when monitoring changed the configuration (e.g. to Charge or when charge was done)
        # During those changes many nodes are killed. We want to ignore these kills since we are only interested in the
        # kills that occured during normal operation
        tp_reconfs = copy.copy(self.nodes_mon["ReconfigureCharging"].ds.index.to_list())
        tp_reconfs.extend(self.nodes_mon["ReconfigureOperating"].ds.index.to_list())
        self.nodes_mon["KillROSNode"].filter_around_timepoints(tp_reconfs, timedelta(minutes=2))
        # If Move base was killed (Navigationr restart) we also do not count this here, because this is evaluated
        # separately
        tp_movebase = copy.copy(self.nodes_mon["KillMoveBase"].ds.index.to_list())
        self.nodes_mon["KillROSNode"].filter_around_timepoints(tp_movebase, timedelta(seconds=15))

        tp_loc = copy.copy(self.nodes_mon["RestartLocalization"].ds.index.to_list())
        self.nodes_mon["KillROSNode"].filter_around_timepoints(tp_loc, timedelta(seconds=25))

        # Check how often the CheckMask BT Nodes were activated and if they were successful
        self.write_nodes_from_bag(self.nodes_mask, bag, '/no_mask_detection/behavior_tree_log')
        print_activations(self.nodes_mask)

        # Check how often the Patrolling BT Nodes were activated and if they were successful
        self.write_nodes_from_bag(self.nodes_patrol, bag, '/patrolling/behavior_tree_log')
        # print_activations(self.nodes_patrol)
        # Calculate number of successful and failed docking attempts
        self.num_docks_successful = self.nodes_patrol["Docked"].get_num_of_activations()
        self.num_docks_failed = self.nodes_patrol["ShutdownRobot"].get_num_of_activations()

        self.calc_time_active()
        self.print_info()

    def print_info(self):
        print("\n-----------------------\n")
        print("Date: " + self.date.strftime("%Y-%m-%d"))
        print("Total time in operation: " + str(self.t_operating))
        print("Total time in failure: " + str(self.t_recovering))
        print("Total time not moving: " + str(self.t_standing))
        print("Total time charging: " + str(self.t_charging))
        if self.perc_active is not None:
            perc_act_str = "{:.1f}".format(self.perc_active)
            print("Percentage moving of operating time: " + perc_act_str + "%")
        print("Traveled distance: " + str(self.odom_dist.get_distance()) + " m")
        print("Successful dockings: " + str(self.num_docks_successful) + ", unsuccessful: " + str(
            self.num_docks_failed))
        if self.tsl_max is not None and self.tsl_avg is not None:
            print("Maximum TSL: " + str(self.tsl_max))
            print("Average TSL: " + str(self.tsl_avg))
            print("Total Eval time: " + str(self.t_eval))
        if self.num_restarts is not None:
            print("Num node restarts: " + str(self.num_restarts) + ", successful: " + str(self.num_restarts_suc/self.num_restarts * 100) + "%")
        print("\n-----------------------\n")

    def write_recoveries_from_bag(self, recoveries, bag, topic):
        print("Progress for topic '" + topic + "'")

        msg_old = MonitoredNavEvent()
        msg_old.event_start_time = rospy.Time(1)
        with tqdm(total=bag.get_message_count(topic_filters=[topic])) as pbar:
            # Loop through the messages
            for _, msg, _ in bag.read_messages(topics=[topic]):
                # We skip the first iteration and save the message since we always need to provide to msg objects
                if msg_old.event_start_time == rospy.Time(1):
                    msg_old = msg
                    continue
                else:
                    # Save the data for the recovery behav
                    for _, recovery in recoveries.items():
                        recovery.save_if_activated(msg_old, msg)
                    msg_old = msg
                pbar.update(1)
            # Also check the last message
            for _, recovery in recoveries.items():
                recovery.save_if_activated(msg_old, None)

    def write_nodes_from_bag(self, nodes, bag, topic):
        # print("Will write node status and data for the following nodes from topic '" + topic + "'")
        # for key in nodes.keys():
        #    print("- " + key)
        print("Progress for topic '" + topic + "'")

        with tqdm(total=bag.get_message_count(topic_filters=[topic])) as pbar:
            # Loop through the messages
            for _, msg, _ in bag.read_messages(topics=[topic]):
                # Set the UID for all nodes
                for _, node in nodes.items():
                    node.set_uid(msg.behavior_tree)
                # Loop through every state change and check which nodes were activated
                for state_change in msg.state_changes:
                    for _, node in nodes.items():
                        node.save_if_activated(state_change)
                pbar.update(1)
            # Convert the data to a timeseries object
            for _, node in nodes.items():
                node.write_df()

    def write_stand_time_from_bag(self, bag, topic):
        print("Progress for topic '" + topic + "'")
        tp_not_nav = np.array([])
        with tqdm(total=bag.get_message_count(topic_filters=[topic])) as pbar:
            # Loop through the messages
            last_value = None
            for _, msg, ts in bag.read_messages(topics=[topic]):
                for info in msg.info:
                    if not info.description == "MoveBase-Monitor":
                        continue
                    for value in info.values:
                        if not value.key == "move_base_monitor_fb":
                            continue
                        if last_value is None:
                            last_value = value
                            continue
                        if value.errorlevel == 0.0 and (last_value.errorlevel == 1.0 or last_value.errorlevel == 0.5):
                           tp_not_nav = np.append(tp_not_nav, float(last_value.value))
                        last_value = value
                pbar.update(1)
            # Filter unlogic values which are unreasonably large due to bug in implementation
            if tp_not_nav.size == 0:
                self.t_standing = timedelta(0)
            else:
                tp_not_nav = tp_not_nav[tp_not_nav  < 400]
                self.t_standing = timedelta(seconds=tp_not_nav.sum())



    def dump(self, folder):
        filename = os.path.join(folder, filename_from_date(self.date))
        pickle.dump(self, open(filename, "wb"))

    def set_tsl_values(self, start, interrupts, end):
        ts = copy.copy(interrupts)
        ts.insert(0, start)
        ts.append(end)
        #Calculate differences between timepoints
        diffs = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
        # Manually reduce the diffs about the weekend days
        diffs[1] -= timedelta(days=2)
        diffs[4] -= timedelta(days=2)
        self.tsl_max = max(diffs)
        self.tsl_avg = sum(diffs, timedelta(0)) / len(diffs)
        self.t_eval = end - start

    def add_weekend_days(self, num_days):
        self.t_charging += timedelta(num_days)

    def calc_time_recovering(self):
        for _, recovery in self.nav_recoveries.items():
            if recovery.name is not "nav_help":
                self.t_recovering += recovery.time_spent

    def calc_time_active(self):
        # When robot is standing: Failure (time in recovery is a part of this time)
        time_in_fail = (self.t_standing).total_seconds()
        time_in_operation = self.t_operating.total_seconds()
        self.perc_active = (time_in_operation - time_in_fail)/time_in_operation * 100

    def calc_node_restarts(self):
        ts_all = self.nodes_mon["KillROSNode"].ds[self.nodes_mon["KillROSNode"].ds.values == 2]
        suc = 0.0
        for i in range(1, len(ts_all.index)):
           if (ts_all.index[i] - ts_all.index[i-1]).total_seconds() > SUCCESS_TIME_TRESH:
               suc += 1.0

        self.num_restarts =  len(ts_all)
        self.num_restarts_suc = suc



    @classmethod
    def load(cls, filename):
        return pickle.load(open(filename, "rb"))
