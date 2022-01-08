#!/usr/bin/env python

#..................................................................................
# Author  :  Saiffullah Sabir Mohamed
# Github  :  https://github.com/TechnicalVillager
# Website :  http://technicalvillager.github.io/
# Source  :  https://github.com/TechnicalVillager/triangular-mission-dronekit-python/
#..................................................................................

# Import Necessary Packages
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, math

def basic_takeoff(altitude):

    """

    This function take-off the vehicle from the ground to the desired
    altitude by using dronekit's simple_takeoff() function.

    Inputs:
        1.  altitude            -   TakeOff Altitude

    """

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(2)
    vehicle.simple_takeoff(altitude)

    while True:
        print("Reached Height = ", vehicle.location.global_relative_frame.alt)

        if vehicle.location.global_relative_frame.alt >= (altitude - 1.5):
            break

def change_mode(mode):

    """

    This function will change the mode of the Vehicle.

    Inputs:
        1.  mode            -   Vehicle's Mode

    """

    vehicle.mode = VehicleMode(mode)

def send_to(latitude, longitude, altitude):

    """

    This function will send the drone to desired location, when the 
    vehicle is in GUIDED mode.

    Inputs:
        1.  latitude            -   Destination location's Latitude
        2.  longitude           -   Destination location's Longitude
        3.  altitude            -   Vehicle's flight Altitude

    """

    if vehicle.mode.name == "GUIDED":
        location = LocationGlobalRelative(latitude, longitude, float(altitude))
        vehicle.simple_goto(location)
        time.sleep(1)

def change_alt(step):

    """
    
    This function will increase or decrease the altitude
    of the vehicle based on the input.

    Inputs:
        1.  step            -   Increase 5 meters of altitude from 
                                current altitude when INC is passed as argument.

                            -   Decrease 5 meters of altitude from 
                                current altitude when DEC is passed as argument.

    """

    actual_altitude = int(vehicle.location.global_relative_frame.alt)
    changed_altitude = [(actual_altitude + 5), (actual_altitude - 5)]

    if step == "INC":
        if changed_altitude[0] <= 50:
            send_to(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, changed_altitude[0])
        else:
            print("Vehicle Reached Maximum Altitude!!!")

    if step == "DEC":
        if changed_altitude[1] >= 5:
            send_to(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, changed_altitude[1])
        else:
            print("Vehicle Reached Minimum Altitude!!!")

def distance_calculation(homeLattitude, homeLongitude, destinationLattitude, destinationLongitude):

    """

    This function returns the distance between two geographiclocations using
    the haversine formula.

    Inputs:
        1.  homeLattitude          -   Home or Current Location's  Latitude
        2.  homeLongitude          -   Home or Current Location's  Longitude
        3.  destinationLattitude   -   Destination Location's  Latitude
        4.  destinationLongitude   -   Destination Location's  Longitude

    """

    # Radius of earth in metres
    R = 6371e3

    rlat1, rlon1 = homeLattitude * (math.pi/180), homeLongitude * (math.pi/180)
    rlat2, rlon2 = destinationLattitude * (math.pi/180), destinationLongitude * (math.pi/180)
    dlat = (destinationLattitude - homeLattitude) * (math.pi/180)
    dlon = (destinationLongitude - homeLongitude) * (math.pi/180)

    # Haversine formula to find distance
    a = (math.sin(dlat/2) * math.sin(dlat/2)) + (math.cos(rlat1) * math.cos(rlat2) * (math.sin(dlon/2) * math.sin(dlon/2)))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    # Distance (in meters)
    distance = R * c

    return distance

def destination_location(homeLattitude, homeLongitude, distance, bearing):

    """

    This function returns the latitude and longitude of the
    destination location, when distance and bearing is provided.

    Inputs:
        1.  homeLattitude       -   Home or Current Location's  Latitude
        2.  homeLongitude       -   Home or Current Location's  Longitude
        3.  distance            -   Distance from the home location
        4.  bearing             -   Bearing angle from the home location

    """

    # Radius of earth in metres
    R = 6371e3

    rlat1, rlon1 = homeLattitude * (math.pi/180), homeLongitude * (math.pi/180)

    d = distance

    #Converting bearing to radians
    bearing = bearing * (math.pi/180)

    rlat2 = math.asin((math.sin(rlat1) * math.cos(d/R)) + (math.cos(rlat1) * math.sin(d/R) * math.cos(bearing)))
    rlon2 = rlon1 + math.atan2((math.sin(bearing) * math.sin(d/R) * math.cos(rlat1)) , (math.cos(d/R) - (math.sin(rlat1) * math.sin(rlat2))))

    #Converting to degrees
    rlat2 = rlat2 * (180/math.pi) 
    rlon2 = rlon2 * (180/math.pi)

    # Lat and Long as an Array
    location = [rlat2, rlon2]

    return location

def triangle_calculation(side_length):

    """

    This function will generate the geographical coordinates (latitudes & longitudes)
    of the triangular (Equilateral Triangle) path with the given side length. The origin or
    reference location for the generation of the triangular trajectory is the vehicle's current location.

    Inputs:
        1.  side_length         -   Side length of the Equilateral Triangle

    """

    # Vehicle's heading and current location
    angle          =  int(vehicle.heading)
    loc            =  (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_relative_frame.alt)

    # Offset Angle
    offset_angle   =  90

    # Decrementing offset angle in the vehicle's heading angle to form the 
    # triangle direction with respect to the vehicle's heading angle.
    angle         -=  offset_angle

    # Declaring a array variable to store
    # the geogrpahical location of triangular points
    final_location =  []

    for count in range(3):
        # Incrementing heading angle
        # Exterior angle of equilateral triangle = 120 degrees
        angle  += 120

        new_loc =  destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = side_length, bearing = angle)
        final_location.append((new_loc[0], new_loc[1], loc[2]))
        loc     =  (new_loc[0], new_loc[1], loc[2])

    return final_location

def triangular_mission(side_length):

    """

    This function retrieves the triangle coordinates from the triangle_calculation()
    function and guides the vehicle to the retrieved points.

    Inputs:
        1.  side_length         -   Side length of the equilateral triangle

    """

    # Retrieving the array of the locations of the triangular path
    locations  =  triangle_calculation(side_length = side_length)

    for location in locations:

        # Send vehicle to the destination
        send_to(latitude = location[0], longitude = location[1], altitude = location[2])

        while True:

            # Distance between the current location of the vehicle and the destination
            distance = distance_calculation(homeLattitude = vehicle.location.global_frame.lat,
                                            homeLongitude = vehicle.location.global_frame.lon,
                                            destinationLattitude  = location[0],
                                            destinationLongitude = location[1])

            if distance <= 1.8:
                break

            time.sleep(2)

def main():

    # Declaring Vehicle as global variable
    global vehicle

    # Connecting the Vehicle
    vehicle = connect('udpin:127.0.0.1:14551', baud=115200)

    # Setting the Heading angle constant throughout flight
    if vehicle.parameters['WP_YAW_BEHAVIOR'] != 0:
        vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
        print("Changed the Vehicle's WP_YAW_BEHAVIOR parameter")

    while True:

        # Getting Input from User
        value = input("Enter your Input:\n").upper()

        if value == 'TAKEOFF':
            basic_takeoff(altitude = 5)

        if value == 'LAND':
            change_mode(mode = value)

        if value == 'INC' or 'DEC':
            change_alt(step = value)

        if value == 'TRIANGLE':
            side = int(input("Enter Side Length of the Triangular Path (in meters):\n"))
            triangular_mission(side_length = side)

if __name__ == "__main__":
    main()