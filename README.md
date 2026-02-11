# Web UI Controlled Robot

## Project Features
- Intuitive web interface for robot control
- Real-time telemetry and feedback
- Customizable commands and settings
- Supports multiple robots

## Code Logic
- The application communicates with the robot over a secure WebSocket connection.
- Control commands are sent from the UI to the robot, and telemetry data is received in real time.
- Each robot acts as a client, maintaining individual states and settings.

## Architecture
- Built on a client-server model:
  - **Frontend**: Developed using HTML, CSS, and JavaScript for a responsive user experience.
  - **Backend**: Node.js server handles WebSocket connections and robot state management.
- Modular design allows easy addition of features and connectivity to various robot hardware.

This project aims to provide a seamless experience for users controlling robots via a web browser, focusing on simplicity and scalability.