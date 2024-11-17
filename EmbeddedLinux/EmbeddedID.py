import firebase_admin
from firebase_admin import credentials, firestore
import json
import os
from datetime import datetime

class EmbeddedDeviceClient:
    #finds the robot_config.json we generate from either the device or main pc depends how we want to ship these globally
    def __init__(self, config_path="robot_config.json"):
        self.config_path = config_path
        self.device_id = None
        self.load_device_id()
        
        #Firebase connection Object basically grab ur firebase ID from firebase.com
        cred = credentials.Certificate("get ur own code and place it here make sure it's filepath if it's in same directory just type file name with .json")
        firebase_admin.initialize_app(cred)
        self.db = firestore.client()
    
    #essentially this function will read the json file and search for 'device_id' which was generated from the main pc
    def load_device_id(self):
        try:
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    config = json.load(f)
                    self.device_id = config.get('device_id')
                    print(f"📱 Loaded device ID: {self.device_id}")
            else:
                print("No device ID found.")
                #will throw an error and the error is e so it tries and if it can't it'll except so it's sort of like a TRY and if can't it'll throw error
        except Exception as e:
            print(f"Error loading device ID: {str(e)}")
    

    #this function will update the sensor data in firebase which should be a file named sensor_data so to navigate devices->devicename->sensor_data->current_data <- that's where the data is stored
    def send_sensor_data(self, sensor_data):
        #just tests if there's a device ID assigned to this class if not will just not try updatnig
        if not self.device_id:
            print("No device ID available")
            return False
            
        try:
            device_ref = self.db.collection('devices').document(self.device_id)
            sensor_ref = device_ref.collection('sensor_data').document('current_data')
            #main format of the data 
            data = {
                'timestamp': firestore.SERVER_TIMESTAMP,
                'data': sensor_data,
                'last_updated': datetime.now().isoformat()
            }
            #updates the current data assure that merge=True so it doesn't overwrite the file completely which might waste requests
            sensor_ref.set(data, merge=True)
            print(f"Sensor data updated successfully")
            return True
            
        except Exception as e:
            print(f"Error sending sensor data: {str(e)}")
            return False
    
    #this function just updates the status of the robot by default it will be 'active' but once the user runs the robot it will set it to 'running'
    def update_status(self, status):
        if not self.device_id:
            return False
            
        try:
            device_ref = self.db.collection('devices').document(self.device_id)
            device_ref.update({
                'last_active': firestore.SERVER_TIMESTAMP,
                'status': status
            })
            print(f"Status updated to: {status}")
            return True
        except Exception as e:
            print(f"Error updating status: {str(e)}")
            return False

#we call our object here and update the sensor data, this is where we'll update the sensor data for MCU sensor data so we need an entirely different code in the MCU serial communication either COMN or usb_tty, etc
if __name__ == "__main__":
    client = EmbeddedDeviceClient()

    #test object
    sensor_data = {
        'temperature': 25.5,
        'battery': 100,
        'position': {'x': 100, 'y': 200}
    }
    #update the status of robot
    client.update_status('running')
    client.send_sensor_data(sensor_data)