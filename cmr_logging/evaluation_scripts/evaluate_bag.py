#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import rosbag
from evaluation_scripts import evaluate_monitoring as em
from datetime import datetime
from tqdm import tqdm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pickle
import os
from multiprocessing import Pool
from geometry_msgs.msg import Pose


def process_bag(bagname):
    bag = rosbag.Bag(os.path.join(folder, bagname))
    data = em.EvaluationData()
    data.set_from_bag(bag)
    data.dump("data")
    bag.close()

write = False
folder = "/home/parallels/rosbags"
bag_list = os.listdir(folder)
data_tot = em.EvaluationData()
if write:

    pool = Pool(4)
    pool.map(process_bag, bag_list)
    #bagname = "lta_eval_2021-03-17-00-00-00.bag"
    #process_bag(bagname)

else:
    start_date = "2021-03-02"
    end_date = "2021-03-18"
    dat_range = pd.date_range(start=start_date, end=end_date)
    for date in dat_range:
        fname = os.path.join("data", em.filename_from_date(date))
        if not os.path.exists(fname):
            continue
        data = em.EvaluationData.load(fname)
        data_tot += data

    start_eval = datetime(2021, 3, 2, 13, 00, 00)
    unplanned_interrupts = [datetime(2021, 3, 4, 13, 55, 00), # Shutdown due to unsuccesful docking attempt
                            datetime(2021, 3, 9, 13, 28, 00), # Bug in battery monitoring lead to non-sending of Charge signal ( so battery would have died later on)
                            datetime(2021, 3, 11, 9, 11, 00), # Program crash cyle (EKF and navigation crashed again and again). Restart of all ros nodes and roscore necessary
                            datetime(2021, 3, 12, 13, 55, 00), # Docked but not charging, manual start of charging process
                            ]
    end_eval = datetime(2021, 3, 18, 8, 30, 59)
    data_tot.set_tsl_values(start_eval, unplanned_interrupts, end_eval)
    data_tot.add_weekend_days(4)
    data_tot.calc_time_active()
    data_tot.calc_node_restarts()
    data_tot.print_info()
    print("Mask detections:")
    em.print_activations(data_tot.nodes_mask)
    print("-------------------\n")
    print("Patrol:")
    em.print_activations(data_tot.nodes_patrol)
    print("-------------------\n")
    print("Monitoring:")
    em.print_activations(data_tot.nodes_mon)
    print("-------------------\n")
    print("Nav recoveries:")
    em.print_nav_activations(data_tot.nav_recoveries)

