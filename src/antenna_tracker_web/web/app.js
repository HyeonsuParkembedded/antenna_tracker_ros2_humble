/* Antenna Tracker Web Dashboard - rosbridge client */

const WS_URL = 'ws://' + window.location.hostname + ':9090';
const MAX_CHART_POINTS = 200;

let ros = null;
let chart = null;
let chartData = { az: [], el: [], tgtAz: [], tgtEl: [], labels: [] };

function connectROS() {
    ros = new ROSLIB.Ros({ url: WS_URL });

    ros.on('connection', function () {
        document.getElementById('statusDot').className = 'status-dot connected';
        document.getElementById('statusText').textContent = 'Connected';
        subscribeTopics();
    });

    ros.on('error', function () {
        document.getElementById('statusDot').className = 'status-dot disconnected';
        document.getElementById('statusText').textContent = 'Error';
    });

    ros.on('close', function () {
        document.getElementById('statusDot').className = 'status-dot disconnected';
        document.getElementById('statusText').textContent = 'Disconnected';
        setTimeout(connectROS, 3000);
    });
}

function subscribeTopics() {
    /* Antenna State */
    var stateTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/antenna/state',
        messageType: 'antenna_tracker_msgs/msg/AntennaState'
    });
    stateTopic.subscribe(function (msg) {
        document.getElementById('curAz').textContent = msg.current_azimuth.toFixed(2) + '\u00B0';
        document.getElementById('curEl').textContent = msg.current_elevation.toFixed(2) + '\u00B0';
        document.getElementById('tgtAz').textContent = msg.target_azimuth.toFixed(2) + '\u00B0';
        document.getElementById('tgtEl').textContent = msg.target_elevation.toFixed(2) + '\u00B0';
        document.getElementById('azErr').textContent = msg.az_error.toFixed(2) + '\u00B0';
        document.getElementById('elErr').textContent = msg.el_error.toFixed(2) + '\u00B0';
        document.getElementById('azMotor').textContent = msg.az_motor_cmd.toFixed(1);
        document.getElementById('elMotor').textContent = msg.el_motor_cmd.toFixed(1);

        updateChart(msg.current_azimuth, msg.current_elevation,
                    msg.target_azimuth, msg.target_elevation);
    });

    /* Diagnostics */
    var diagTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/antenna/diagnostics',
        messageType: 'antenna_tracker_msgs/msg/TrackerDiagnostics'
    });
    diagTopic.subscribe(function (msg) {
        setHealth('healthImu', msg.imu_ok);
        setHealth('healthMag', msg.mag_ok);
        setHealth('healthGps', msg.gps_ok);
        setHealth('healthEncoder', msg.encoder_ok);
        setHealth('healthCan', msg.can_ok);

        var cpuEl = document.getElementById('healthCpu');
        cpuEl.textContent = 'CPU: ' + msg.cpu_temp_c.toFixed(1) + '\u00B0C';
        cpuEl.className = 'health-item ' + (msg.cpu_temp_c < 80 ? 'ok' : 'fail');

        document.getElementById('loopRate').textContent = msg.loop_rate_hz.toFixed(1) + ' Hz';
    });

    /* Target GPS */
    var gpsTopics = new ROSLIB.Topic({
        ros: ros,
        name: '/antenna/target_gps',
        messageType: 'antenna_tracker_msgs/msg/TargetGPS'
    });
    gpsTopics.subscribe(function (msg) {
        document.getElementById('tgtLat').textContent = msg.latitude.toFixed(6);
        document.getElementById('tgtLon').textContent = msg.longitude.toFixed(6);
        document.getElementById('tgtAlt').textContent = msg.altitude_m.toFixed(1) + ' m';
        document.getElementById('tgtRssi').textContent = msg.rssi_dbm.toFixed(0) + ' dBm';
    });
}

function setHealth(elementId, ok) {
    var el = document.getElementById(elementId);
    el.className = 'health-item ' + (ok ? 'ok' : 'fail');
}

function setMode(mode) {
    if (!ros || !ros.isConnected) return;

    var modeNames = ['AUTO', 'MANUAL', 'STANDBY', 'EMERGENCY'];
    document.getElementById('curMode').textContent = modeNames[mode] || 'UNKNOWN';

    var client = new ROSLIB.Service({
        ros: ros,
        name: '/antenna/set_mode',
        serviceType: 'antenna_tracker_msgs/srv/SetMode'
    });

    var request = new ROSLIB.ServiceRequest({ mode: mode });
    client.callService(request, function (result) {
        if (!result.success) {
            alert('Mode change failed: ' + result.message);
        }
    });
}

function initChart() {
    var ctx = document.getElementById('trackingChart').getContext('2d');
    chart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {
                    label: 'Azimuth',
                    data: [],
                    borderColor: '#00e5ff',
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false
                },
                {
                    label: 'Elevation',
                    data: [],
                    borderColor: '#e94560',
                    borderWidth: 1.5,
                    pointRadius: 0,
                    fill: false
                },
                {
                    label: 'Target AZ',
                    data: [],
                    borderColor: '#00e5ff',
                    borderDash: [5, 5],
                    borderWidth: 1,
                    pointRadius: 0,
                    fill: false
                },
                {
                    label: 'Target EL',
                    data: [],
                    borderColor: '#e94560',
                    borderDash: [5, 5],
                    borderWidth: 1,
                    pointRadius: 0,
                    fill: false
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            scales: {
                x: { display: false },
                y: {
                    min: -10,
                    max: 370,
                    ticks: { color: '#888' },
                    grid: { color: '#333' }
                }
            },
            plugins: {
                legend: {
                    labels: { color: '#ccc', boxWidth: 12 }
                }
            }
        }
    });
}

function updateChart(az, el, tgtAz, tgtEl) {
    var now = new Date().toLocaleTimeString();

    chart.data.labels.push(now);
    chart.data.datasets[0].data.push(az);
    chart.data.datasets[1].data.push(el);
    chart.data.datasets[2].data.push(tgtAz);
    chart.data.datasets[3].data.push(tgtEl);

    if (chart.data.labels.length > MAX_CHART_POINTS) {
        chart.data.labels.shift();
        chart.data.datasets.forEach(function (ds) { ds.data.shift(); });
    }

    chart.update();
}

/* Init */
initChart();
connectROS();
