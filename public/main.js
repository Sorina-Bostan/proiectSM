let dataInterval = null;

function updateDisplay(data) {
    document.getElementById('puls').textContent = data.bpm !== undefined ? data.bpm : '-';
    document.getElementById('saturatie').textContent = data.spo2 !== undefined ? data.spo2 : '-';
}

function fetchData() {
    fetch('/data')
        .then(response => response.json())
        .then(data => updateDisplay(data))
        .catch(() => {
            updateDisplay({bpm: '-', spo2: '-'});
        });
}

function start() {
    fetch('/start')
        .then(() => {
            if (dataInterval) clearInterval(dataInterval);
            fetchData();
            dataInterval = setInterval(fetchData, 1000);
        });
}

function stop() {
    fetch('/stop')
        .then(() => {
            if (dataInterval) clearInterval(dataInterval);
            dataInterval = null;
        });
}

document.addEventListener('DOMContentLoaded', fetchData);