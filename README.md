# ros_database
small scripts to connect to and run queries on a MySQL database

## MySQL

    - MySQL must be installed (at least version 8.4.5)
    - Install the MySQL Python connector: `pip3 install mysql-connector-python`
    - Set the user and password as environment variables:
      - export DB_USER=<username>
      - export DB_PASSWORD=<password>
## ROS Package
    - colcon build --packages-select database_connector
    - ros2 run database_connector main.py

## To Do
    - For new systems, update the Python script to install the database, connect to it, and insert data.
