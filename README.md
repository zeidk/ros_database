# ros_database
small scripts to connect to and run queries on a MySQL database

## MySQL

    - MySQL must be installed (at least version 8.4.5)
    - Install the MySQL Python connector:
```bash
$ pip3 install mysql-connector-python
Collecting mysql-connector-python
  Downloading mysql_connector_python-9.3.0-cp310-cp310-manylinux_2_28_x86_64.whl (33.8 MB)
     ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 33.8/33.8 MB 6.2 MB/s eta 0:00:00
Successfully installed mysql-connector-python-9.3.0
```
    - Set the user and password as environment variables:
      - export DB_USER=<username>
      - export DB_PASSWORD=<password>
## ROS Package
    - colcon build --packages-select database_connector
    - ros2 run database_connector main.py

## To Do
    - For new systems, update the Python script to install the database, connect to it, and insert data.
