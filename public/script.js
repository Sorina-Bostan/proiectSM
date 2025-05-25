// public/script.js
let dataInterval; // Stores the ID of the interval for polling
let stopTimeout;  // Stores the ID for a timeout to stop polling if no finger
const POLLING_INTERVAL_MS = 1000; // Fetch data every 1 second
const NO_FINGER_STOP_DELAY_MS = 10000; // Stop polling after 10s if no finger is detected

// Get references to the HTML elements once
const pulsElement = document.getElementById('puls');
const saturatieElement = document.getElementById('saturatie');
// Optional: Add an element with id="statusMessage" in your HTML to display status
// const statusMessageElement = document.getElementById('statusMessage');

// Function to change color based on values
function verde_rosu() {
    if (!pulsElement || !saturatieElement) return; // Elements not found

    const pText = pulsElement.textContent;
    const sText = saturatieElement.textContent;

    // Try to parse BPM, removing " BPM" if present
    const pMatch = pText.match(/(\d+)/);
    const p = pMatch ? parseInt(pMatch[1]) : NaN;

    // Try to parse SpO2, removing "%" if present
    const sMatch = sText.match(/(\d+(\.\d+)?)/); // Allows for decimals like 98.5%
    const s = sMatch ? parseFloat(sMatch[1]) : NaN;
    
    if (!isNaN(p)) {
        if (p > 100 || p < 50) { // Typical adult resting HR is 60-100. Adjust if needed.
            pulsElement.style.color = "red";
        } else {
            pulsElement.style.color = "green";
        }
    } else {
        pulsElement.style.color = "black"; // Default color if not a number
    }

    if (!isNaN(s)) {
        saturatieElement.style.color = s < 92 ? "red" : "green"; // SpO2 < 92% is often a concern
    } else {
        saturatieElement.style.color = "black"; // Default color
    }
}

async function fetchData() {
    try {
        const response = await fetch('/data');
        if (!response.ok) {
            console.error('Error fetching data:', response.status);
            if (saturatieElement) saturatieElement.textContent = 'Error';
            if (pulsElement) pulsElement.textContent = 'Error';
            // if (statusMessageElement) statusMessageElement.textContent = 'Connection Error';
            verde_rosu(); // Update colors even on error (will likely be black)
            return;
        }
        const data = await response.json();

        if (data.active) {
            if (data.finger_on) {
                if (saturatieElement) saturatieElement.textContent = data.spo2 > 0 ? data.spo2.toFixed(1) + '%' : 'Calculating...';
                if (pulsElement) pulsElement.textContent = data.bpm > 0 ? data.bpm.toFixed(0) + ' BPM' : 'Calculating...';
                
                // Reset the no-finger stop timeout if a finger is detected
                if (stopTimeout) {
                    clearTimeout(stopTimeout);
                    stopTimeout = null;
                }
            } else { // Finger not on, but measurement is active
                if (saturatieElement) saturatieElement.textContent = '---';
                if (pulsElement) pulsElement.textContent = '---';
                // Start a timeout to stop polling if finger remains off
                if (!stopTimeout && dataInterval) { // Only start if polling is active and no timeout yet
                    console.log("No finger detected while active, starting stop timeout...");
                    stopTimeout = setTimeout(() => {
                        console.log("Stopping due to prolonged no-finger detection.");
                        stopMeasurement(); // Call your existing stop function
                    }, NO_FINGER_STOP_DELAY_MS);
                }
            }
            // if (statusMessageElement) statusMessageElement.textContent = data.status;
        } else { // Measurement not active (stopped by user)
            if (saturatieElement) saturatieElement.textContent = 'Stopped';
            if (pulsElement) pulsElement.textContent = 'Stopped';
            // if (statusMessageElement) statusMessageElement.textContent = data.status || 'Measurement Stopped';
        }

        verde_rosu(); // Update colors AFTER setting text content

    } catch (error) {
        console.error('Failed to fetch data:', error);
        if (saturatieElement) saturatieElement.textContent = 'N/A';
        if (pulsElement) pulsElement.textContent = 'N/A';
        // if (statusMessageElement) statusMessageElement.textContent = 'Fetch Failed';
        verde_rosu(); // Update colors
    }
}

// This is your START function, now integrated with fetching real data
async function start() {
    console.log("Start button clicked");
    try {
        const response = await fetch('/start'); // Tell Pico to start measuring
        const result = await response.json();
        console.log('Start command sent to Pico:', result);

        if (result.status === 'started') {
            if (saturatieElement) saturatieElement.textContent = 'Starting...';
            if (pulsElement) pulsElement.textContent = 'Starting...';
            // if (statusMessageElement) statusMessageElement.textContent = "Measurement Started. Place finger.";
            
            if (stopTimeout) { // Clear any existing no-finger stop timeout
                clearTimeout(stopTimeout);
                stopTimeout = null;
            }
            if (dataInterval) { // Clear existing interval if any (e.g., if start is clicked multiple times)
                clearInterval(dataInterval);
            }
            
            fetchData(); // Fetch data immediately once
            dataInterval = setInterval(fetchData, POLLING_INTERVAL_MS); // Then poll regularly
        }
    } catch (error) {
        console.error('Error sending start command to Pico:', error);
        // if (statusMessageElement) statusMessageElement.textContent = "Error starting.";
    }
}

// This is your STOP function
async function stop() {
    console.log("Stop button clicked");
    try {
        const response = await fetch('/stop'); // Tell Pico to stop measuring
        const result = await response.json();
        console.log('Stop command sent to Pico:', result);

        if (dataInterval) {
            clearInterval(dataInterval);
            dataInterval = null;
        }
        if (stopTimeout) { // Clear no-finger stop timeout as well
            clearTimeout(stopTimeout);
            stopTimeout = null;
        }

        if (saturatieElement) saturatieElement.textContent = 'Stopped';
        if (pulsElement) pulsElement.textContent = 'Stopped';
        // if (statusMessageElement) statusMessageElement.textContent = "Measurement Stopped.";
        verde_rosu(); // Update colors (likely to black or based on "Stopped")
    } catch (error) {
        console.error('Error sending stop command to Pico:', error);
    }
}

// Initialize display and colors on page load
document.addEventListener("DOMContentLoaded", () => {
    if (saturatieElement) saturatieElement.textContent = '---';
    if (pulsElement) pulsElement.textContent = '---';
    verde_rosu(); // Set initial colors (will likely be default/black)

    // You might want to fetch initial status to see if measurement was already active
    // For example:
    // fetchData(); 
    // This would show "Calculating..." or actual values if the Pico was already running
    // However, it's often cleaner to require the user to press "Start".
});

// Make functions available to HTML onclick attributes if you are using them that way
window.start = start;
window.stop = stop;