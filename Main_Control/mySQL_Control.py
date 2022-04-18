import mysql.connector
import socket
import time
import datetime;
import pytz



sql = mysql.connector.connect(
    host="nilerobot.info",
    port="3306",
    user="nilepython",
    password="Trolley123!",
    database="NILE",
    autocommit=True
)

cursor = sql.cursor()


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

def assign_ip():
	
    date_time = datetime.datetime.now().strftime("%m/%d/%Y %H:%M:%S")
    #sql.reconnect()
    
    ip = get_ip()
    cursor.execute("INSERT INTO system_info (ip_address) VALUES ('" + str(ip) + "')")
    sql.commit()
    
    return 0

def pull_next_command():
	time.sleep(0.1)
	try:
		cursor.execute("SELECT * FROM queued_commands ORDER BY timestamp ASC LIMIT 1")
		result = cursor.fetchone()
		id = result[0]
		timestamp = result[1]
		command = result[2]
		theta = result[3]
		r = result[4]
		z = result[5]
		d0 = result[6]
		d1 = result[7]
		i0 = result[8]
		return [id, timestamp, command, theta, r, z, d0, d1, i0]
	except:
		return [0, 0, 0, 0, 0, 0, 0, 0, 0]


			

def time_until():
	ct = datetime.datetime.now()
	result = pull_next_command()
	if (result[0] == 0):
		return -999
	else:
		duration =  ct - result[1]
		duration_s = duration.total_seconds()

		return duration_s

def complete_command(theta, r, z, msg):
	sql.reconnect()
	result = pull_next_command()
	#sql.reconnect()
	time.sleep(0.1)
	cursor.execute("INSERT INTO completed_commands (timestamp_q, command, pos_theta, pos_r, pos_z, pos_theta_q, pos_r_q, pos_z_q, argument_d0, argument_d1, argument_i0, message) VALUES  ("+result[1].strftime("'%Y/%m/%d %H:%M:%S'")+", '"+result[2]+"', "+str(theta)+", "+str(r)+", "+str(z)+", "+str(result[3])+", "+str(result[4])+", "+str(result[5])+", "+str(result[6])+", "+str(result[7])+", "+str(result[8])+", '"+msg+"');")
	sql.commit()
	cursor.execute("DELETE FROM queued_commands WHERE id="+str(result[0]))
	sql.commit()
	
	return 0




def publish_pos(theta, r, z):
	sql.reconnect()
	
	sqlstr = "INSERT INTO robot_status(pos_theta, pos_r, pos_z) VALUES (" + str(theta) + ", " + str(r) + ", " + str(z) + ")"
	cursor.execute(sqlstr)
	sql.commit()
	
	return 0

def publish_soil_sample(theta, r, z, moisture, temp):
	sql.reconnect()
	
	sqlstr = "INSERT INTO soil_samples(pos_theta, pos_r, pos_z, moisture, temp) VALUES (" + str(theta) + ", " + str(r) + ", " + str(z) + ", " + str(moisture) + ", " + str(temp) +")"
	cursor.execute(sqlstr)
	sql.commit()
	
	return 0
