from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
from pymavlink import mavutil
import time
import math
import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import transform
import detect_final
import traceback
import leader
import detect2
# Connect to the Vehicle
vehicle = connect('/dev/ttyACM0', wait_ready=True)
vehicle.mode="GUIDED"
init_loc=LocationGlobalRelative(26.1905741,91.696874,0.5)
#cur_loc=init_loc

def goto(dNorth, dEast,dalt, gotoFunction=vehicle.simple_goto):
    
    currentLocation=vehicle.location.global_relative_frame
    targetLocation=get_location_metres(currentLocation, dNorth, dEast,dalt)
    targetDistance=get_distance_metres(currentLocation, targetLocation)
    vehicle.groundspeed=2
    gotoFunction(targetLocation)
    vehicle.groundspeed=2

#    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
 #       remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
  #      print( "Distance to target: ", remainingDistance)
   #     if remainingDistance<=targetDistance*0.05: #Just below target, in case of undershoot.
    #        print( "Reached target")
     #       break
      #  time.sleep(2)
  
def get_distance_metres(aLocation1, aLocation2):
    
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
   
   
def goto_position_target_global_int(aLocation):
    
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        int(aLocation.lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(aLocation.lon*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
        int(aLocation.alt), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_A>
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
def condition_yaw(vel):
   
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        0,    # param 1, yaw in degrees
        vel,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def rotate_drone(yaw_rate):
    # Create a message for attitude control
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # Timestamp in milliseconds since system boot        1,  # Target system
        1,  # Target component
        0b00000100,  # Mask for yaw rate
        [0, 0, 0],  # Attitude quaternion (roll, pitch, yaw)
        0,  # Body roll rate in radians per second
        0,  # Body pitch rate in radians per second
        yaw_rate,  # Body yaw rate in radians per second
        0   # Thrust
    )

    # Send the message to the vehicle
    vehicle.send_mavlink(msg)


def get_location_metres(original_location, dNorth, dEast,dalt):

    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    newalt=original_location.alt + dalt
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,newalt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,newalt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;
    

def reverse_get_location_metres(original_location, target_location):
    earth_radius = 6378137.0 # Radius of "spherical" earth

    # Change in latitude and longitude in radians
    dLat = (target_location.lat - original_location.lat) * math.pi / 180
    dLon = (target_location.lon - original_location.lon) * math.pi / 180

    # Change in meters
    dNorth = dLat * earth_radius
    dEast = dLon * earth_radius * math.cos(math.pi * original_location.lat / 180)
    dalt=target_location.alt - original_location.alt

    return dNorth, dEast,dalt
    
def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break

        time.sleep(1)
#    cur_loc=LocationGlobalRelative(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.alt)



try:
    target_altitude=2
    vehicle.groundspeed=2
    aruco=0
    vehicle.mode = VehicleMode("GUIDED")


    arm_and_takeoff(target_altitude)
    time.sleep(1)
    while True:
       # rotate_drone(0.3)
       # set_yaw_velocity(0.3)
 #       condition_yaw(20)
        time.sleep(2)
       # t=detect_final.detect()
        ids,x,y,z=detect_final.detect_markers()

        x=round(x/1000,2)
        y=round(y/1000,2)
        z=round(z/1000,2)
        ids=int(ids)
        if ids==aruco:
           # rotate_drone(0)
#            condition_yaw(0)
            cur_loc=vehicle.location.global_relative_frame
            n,e,a=reverse_get_location_metres(init_loc, cur_loc)
            theta_x=- math.pi/2
            theta_y= 0
            theta_z=-1*( vehicle.attitude.yaw)
             #east-x,north-y,alt-z
            east_n,north_n,alt_n=transform.transform_point(x,y,z,0,0,0,theta_x,theta_y,theta_z,1,1,1)
            #vehicle.groundspeed=2
            print(f"Aruco detected.Going to east:{east_n},north:{north_n},alt:{alt_n}")
#            time.sleep(2)
            goto(north_n,east_n,alt_n)
#            time.sleep(2)
           # time.sleep(2)
        else:
            
            continue    
except Exception as e:
    print("Error occurred:", str(e))
    tb = traceback.format_exc()
    print("Traceback details:")
    print(tb) 
    print("Landing....")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(10)    
    
#except KeyboardInterrupt:
#    print("Keyboard interrupt detected. Landing.....")
#    print("Returning to launch")
#    vehicle.mode = VehicleMode("RTL")
#    time.sleep(10)
finally:
    # Disarm the vehicle
    print("Disarming...")
    vehicle.armed = False

    # Close vehicle object
    vehicle.close()

         
         
         

