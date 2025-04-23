import rclpy
from rclpy.node import Node
import mysql.connector
from mysql.connector import Error
import os


class DatabaseNodePython(Node):
    def __init__(self):
        super().__init__("database_node_python")

        self._trial_run_id = 1
        # Get database credentials from environment variables
        self.db_config = {
            "host": os.environ.get("DB_HOST", "localhost"),
            "database": os.environ.get("DB_NAME", "ariac2025_db"),
            "user": os.environ.get("DB_USER", ""),
            "password": os.environ.get("DB_PASSWORD", ""),
        }

        # Check if credentials are available
        if not self.db_config["user"] or not self.db_config["password"]:
            self.get_logger().error(
                "Database credentials not found in environment variables"
            )
            self.get_logger().error(
                "Please set DB_USER and DB_PASSWORD environment variables"
            )
            return

        self.connection = None
        self.connect_to_database()

        # Create a timer for periodic database operations
        self.timer = self.create_timer(3.0, self.timer_callback)

        self.get_logger().info("Database node initialized")

    def connect_to_database(self):
        try:
            self.connection = mysql.connector.connect(**self.db_config)
            if self.connection.is_connected():
                db_info = self.connection.get_server_info()
                self.get_logger().info(f"Connected to MySQL Server version {db_info}")
                cursor = self.connection.cursor()
                cursor.execute("SELECT DATABASE();")
                record = cursor.fetchone()
                self.get_logger().info(f"Connected to database: {record[0]}")
                cursor.close()
        except Error as e:
            self.get_logger().error(f"Error connecting to MySQL database: {e}")

    def disconnect_from_database(self):
        if self.connection is not None and self.connection.is_connected():
            self.connection.close()
            self.get_logger().info("MySQL connection closed")

    def timer_callback(self):
        self.get_logger().info("Executing periodic database operation")
        self.query_all_trials()
        self.calculate_and_update_total_score(self._trial_run_id)
        if self._trial_run_id < 25:
            self._trial_run_id += 1

    def query_all_trials(self):
        if not self.connection or not self.connection.is_connected():
            self.connect_to_database()

        try:
            cursor = self.connection.cursor(dictionary=True)
            query = "SELECT * FROM Trial"
            cursor.execute(query)
            records = cursor.fetchall()

            for row in records:
                self.get_logger().info(
                    f"Trial ID: {row['trial_ID']}, Seed: {row['seed']}, Time Limit: {row['time_limit']}"
                )

            cursor.close()
        except Error as e:
            self.get_logger().error(f"Error while querying data: {e}")

    def query_competitor_scores(self, competitor_id):
        if not self.connection or not self.connection.is_connected():
            self.connect_to_database()

        try:
            cursor = self.connection.cursor(dictionary=True)
            query = """
            SELECT c.team_name, t.trial_ID, trs.total_score, trs.agility_score, trs.time_score, trs.cost_score 
            FROM Competitor c
            JOIN Trial_Run tr ON c.id = tr.competitor_id
            JOIN Trial_Run_Score trs ON tr.id = trs.trial_run_id
            JOIN Trial t ON tr.trial_id = t.id
            WHERE c.id = %s
            ORDER BY t.trial_ID, tr.run_number
            """
            cursor.execute(query, (competitor_id,))
            records = cursor.fetchall()

            for row in records:
                self.get_logger().info(
                    f"Team: {row['team_name']}, Trial: {row['trial_ID']}, "
                    f"Score: {row['total_score']}, Agility: {row['agility_score']}"
                )

            cursor.close()
        except Error as e:
            self.get_logger().error(f"Error while querying competitor scores: {e}")

    def add_new_competitor(self, team_name):
        if not self.connection or not self.connection.is_connected():
            self.connect_to_database()

        try:
            cursor = self.connection.cursor()
            query = "INSERT INTO Competitor (team_name) VALUES (%s)"
            cursor.execute(query, (team_name,))
            self.connection.commit()
            competitor_id = cursor.lastrowid
            self.get_logger().info(
                f"Added new competitor: {team_name}, ID: {competitor_id}"
            )
            cursor.close()
            return competitor_id
        except Error as e:
            self.get_logger().error(f"Error while adding new competitor: {e}")
            return None

    def calculate_and_update_total_score(self, trial_run_id):
        try:
            cursor = self.connection.cursor(dictionary=True)

            # Get basic points total
            cursor.execute(
                """
                SELECT COALESCE(SUM(value), 0) as basic_points 
                FROM Basic_Point WHERE trial_run_id = %s
            """,
                (trial_run_id,),
            )
            basic_points = cursor.fetchone()["basic_points"]

            # Get bonus points total
            cursor.execute(
                """
                SELECT COALESCE(SUM(value), 0) as bonus_points 
                FROM Bonus_Point WHERE trial_run_id = %s
            """,
                (trial_run_id,),
            )
            bonus_points = cursor.fetchone()["bonus_points"]

            # Get penalty points total
            cursor.execute(
                """
                SELECT COALESCE(SUM(value), 0) as penalty_points 
                FROM Penalty_Point WHERE trial_run_id = %s
            """,
                (trial_run_id,),
            )
            penalty_points = cursor.fetchone()["penalty_points"]

            # Get agility score
            cursor.execute(
                """
                SELECT COALESCE(AVG(value), 0) as agility_score 
                FROM Agility_Score WHERE trial_run_id = %s
            """,
                (trial_run_id,),
            )
            agility_score = cursor.fetchone()["agility_score"]

            # Get time score (simplified calculation)
            cursor.execute(
                """
                SELECT 
                    TIMESTAMPDIFF(SECOND, start_time, completion_time) as time_taken,
                    t.time_limit
                FROM Trial_Run tr
                JOIN Trial t ON tr.trial_id = t.id
                WHERE tr.id = %s
            """,
                (trial_run_id,),
            )
            time_result = cursor.fetchone()

            time_taken = time_result["time_taken"] if time_result["time_taken"] else 0
            time_limit = time_result["time_limit"] if time_result["time_limit"] else 300

            time_score = 100 * (1 - time_taken / time_limit) if time_limit > 0 else 0
            time_score = max(0, time_score)  # Ensure non-negative

            # Get competitor's sensor cost for cost score
            cursor.execute(
                """
                SELECT c.total_sensor_cost
                FROM Trial_Run tr
                JOIN Competitor c ON tr.competitor_id = c.id
                WHERE tr.id = %s
            """,
                (trial_run_id,),
            )
            cost_result = cursor.fetchone()

            sensor_cost = (
                cost_result["total_sensor_cost"]
                if cost_result["total_sensor_cost"]
                else 0
            )
            cost_score = 100 * (1 - sensor_cost / 1000)  # Assuming 1000 is max cost
            cost_score = max(0, cost_score)  # Ensure non-negative

            # Calculate total score (simplified formula)
            total_score = (
                basic_points
                + bonus_points
                - penalty_points
                + agility_score
                + time_score
                + cost_score
            )

            # Insert or update score
            cursor.execute(
                """
                INSERT INTO Trial_Run_Score 
                (trial_run_id, total_score, agility_score, time_score, cost_score) 
                VALUES (%s, %s, %s, %s, %s)
                ON DUPLICATE KEY UPDATE 
                total_score = VALUES(total_score),
                agility_score = VALUES(agility_score),
                time_score = VALUES(time_score),
                cost_score = VALUES(cost_score),
                timestamp = CURRENT_TIMESTAMP
            """,
                (trial_run_id, total_score, agility_score, time_score, cost_score),
            )

            self.connection.commit()
            self.get_logger().info(
                f"Updated total score for run {trial_run_id}: {total_score}"
            )

            cursor.close()
            return total_score
        except Error as e:
            self.get_logger().error(f"Error calculating total score: {e}")
            return None

    def get_cells_for_trial(self, trial_id):
        try:
            cursor = self.connection.cursor(dictionary=True)
            query = """
            SELECT c.* 
            FROM Cell c
            JOIN Trial t ON c.trial_id = t.id
            WHERE t.trial_ID = %s
            """
            cursor.execute(query, (trial_id,))
            cells = cursor.fetchall()

            for cell in cells:
                self.get_logger().info(
                    f"Cell ID: {cell['id']}, Name: {cell['name']}, "
                    f"Voltage: {cell['voltage']}, Rotation: {cell['rotation']}, "
                    f"Is Defect: {cell['is_defect']}, Defect Type: {cell['defect_type']}"
                )

            cursor.close()
            return cells
        except Error as e:
            self.get_logger().error(f"Error retrieving cells for trial: {e}")
            return None

    def destroy_node(self):
        self.disconnect_from_database()
        super().destroy_node()
