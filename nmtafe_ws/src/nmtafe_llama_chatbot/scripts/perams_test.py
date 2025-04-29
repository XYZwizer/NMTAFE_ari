#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys
import argparse
if __name__ == "__main__":
    #get any arguments passed to the script
    args = rospy.myargv(argv=sys.argv)
    print("rospy args", args)
    parser = argparse.ArgumentParser(description="Run SAM2 Image Predictor")
    parser.add_argument("data", type=str, help="Input Data Folder")
    args = parser.parse_args()
    print("argspass args", args)
    