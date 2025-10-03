import carla
import time
import os
import csv

#-------------------
#     Setups
#------------------

# Output directories
os.makedirs('output/camera', exist_ok=True)
os.makedirs('output/lidar', exist_ok=True)
os.makedirs('output/logs', exist_ok=True)

# CSV files for GNSS and IMU
gnss_file = open('output/logs/gnss.csv', 'w', newline='')
imu_file = open('output/logs/imu.csv', 'w', newline='')
gnss_writer = csv.writer(gnss_file)
imu_writer = csv.writer(imu_file)
gnss_writer.writerow(['frame', 'latitude', 'longitude', 'altitude'])
imu_writer.writerow(['frame', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'compass'])

client= carla.Client('localhost',2000)
client.set_timeout(10.0) #the time to wait it until carla silmulation open
world = client.get_world()

#-------------------
#  Synchronous mode
#-------------------
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05  # 20 FPS simulation
world.apply_settings(settings)

#---------------
#  set weather
#--------------
weather= carla.WeatherParameters(
                   cloudiness=20.0,
                   precipitation=20.0,
                   sun_altitude_angle=30)

#-------------------
#  Spawing vehicle
#------------------
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('model3')[0]
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp , spawn_point)
# Move vehicle forward automatically
vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0))

#-----------------
# Adding sensors
#---------------- 
sensors = []
#---RGB Camera---
camera_rgb = blueprint_library.find('sensor.camera.rgb')
camera_rgb.set_attribute('image_size_x', '800')
camera_rgb.set_attribute('image_size_y', '600')
camera_rgb.set_attribute('fov','90')

camera_spawn_point = carla.Transform(carla.Location(x=4, z=1.6),carla.Rotation(roll=0,pitch=0, yaw=0))
camera= world.spawn_actor(camera_rgb, camera_spawn_point , attach_to=vehicle)
sensors.append(camera)

camera.listen(lambda image: image.save_to_disk('output/camera_%06d.png' % image.frame))

#---LiDAR---
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('range', '100.0')
lidar_bp.set_attribute('rotation_frequency', '10')
lidar_bp.set_attribute('channels', '32')
lidar_bp.set_attribute('points_per_second', '56000')

lidar_spawn_point = carla.Transform(carla.Location(x=0, y=0, z=2.5))

lidar = world.spawn_actor(lidar_bp, lidar_spawn_point, attach_to=vehicle) 
sensors.append(lidar)

lidar.listen(lambda point_cloud: point_cloud.save_to_disk('output/lidar_%06d.ply' % point_cloud.frame))

#---GNSS---
gnss_bp = blueprint_library.find('sensor.other.gnss')
gnss_spawn_point = carla.Transform(carla.Location(x=0, y=0, z=2))

gnss = world.spawn_actor(gnss_bp, gnss_spawn_point, attach_to=vehicle)
sensors.append(gnss)

gnss.listen(lambda event: print (f"GNSS: {event.latitude}, {event.longitude}, {event.altitude}"))
#---IMU---
imu_bp = blueprint_library.find('sensor.other.imu')
imu_spawn_point = carla.Transform(carla.Location(x=0, y=0, z=2))

imu = world.spawn_actor(imu_bp, imu_spawn_point, attach_to=vehicle)
sensors.append(imu)

imu.listen(lambda event: print(f"IMU: acc={event.accelerometer}, gyro={event.gyroscope}, compass={event.compass}"))

# -------------------
#   Simulation loop
# --------------------
print("Running simulation... press Ctrl+C to exit.")

try:
    while True:
        world.tick()  # synchronous tick
except KeyboardInterrupt:
    print("\nStopping simulation...")

finally:
    # Destroy actors
    print("Destroying sensors and vehicle...")
    for sensor in sensors:
        sensor.destroy()
    vehicle.destroy()
    print("Actors destroyed. Closing CSV files.")
    gnss_file.close()
    imu_file.close()
    # Restore asynchronous mode
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    print("Simulation cleaned up successfully.")
