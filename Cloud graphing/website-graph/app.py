from flask import Flask, jsonify, request, send_from_directory
import os


# SETUP COMMANDS:
# sudo yum update -y
# sudo yum install python3 -y
# sudo yum install python3pip -y
# sudo pip3 install flask
# RUN COMMAND:
# python3 app.py on EC2 instance


app = Flask(__name__)

# In-memory data storage (to simulate the sensor data)
data_storage = []

# Route to serve the HTML page
@app.route('/')
def serve_graph():
    return send_from_directory('static', 'index.html')

# Endpoint to get the latest data
@app.route('/api/data', methods=['GET'])
def get_data():
    if len(data_storage) > 0:
        return jsonify(data_storage[-1])  # Return the latest data
    else:
        return jsonify({"message": "No data available"}), 404

# Endpoint to receive data from Arduino (via POST)
@app.route('/api/data', methods=['POST'])
def receive_data():
    content = request.get_json()
    if content:
        data_storage.append(content)  # Store the received data
        return jsonify({"message": "Data received successfully!"}), 200
    else:
        return jsonify({"message": "Invalid data"}), 400

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)  # Listen on all interfaces for access from the web
