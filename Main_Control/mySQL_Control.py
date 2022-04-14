import mysql.connector
import socket

sql = mysql.connector.connect(
    host="nilerobot.info",
    port="3306",
    user="nilepython",
    password="Trolley123!",
    database="NILE"
)

# Function to get ip address
def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


def pull_next_command():
    cursor = sql.cursor()
    cursor.execute("SELECT * FROM queued_commands ORDER BY timestamp ASC LIMIT 1")
    result = cursor.fetchall()
    print(result)
    id = result[0][0]
    cursor.close()

pull_next_command()
