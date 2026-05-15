/**
 * 1. 전역 변수 및 ROS 설정
 */
const ros = new ROSLIB.Ros({
    url: 'ws://' + window.location.hostname + ':9090'
});

// 제어 토픽 (로봇 0 제어)
const cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/robot_0/cmd_vel', // 제어 명령은 로봇 0으로 전송
    messageType: 'geometry_msgs/Twist'
});

// 레이턴시 토픽 (로봇 1의 레이턴시 구독)
const latencyEcho = new ROSLIB.Topic({
    ros: ros,
    name: '/robot_1/latency_echo', // 레이턴시 피드백은 로봇 1에서 수신
    messageType: 'std_msgs/Float64'
});

/**
 * 2. 명령 전송 함수 (전역 스코프)
 * HTML 버튼의 onclick="sendCommand('w')" 에서 직접 호출 가능
 */
function sendCommand(key) {
    let twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,// Date.now(), // RTT 측정을 위한 현재 타임스탬프
            z: 0
        },
        angular: { x: 0, y: 0, z: 0 }
    });

    let cmdText = "STOP";
    const keyLower = key.toLowerCase();

    switch(keyLower) {
        case 'w': twist.linear.x = 0.5; cmdText = "FORWARD"; break;
        case 's': twist.linear.x = -0.5; cmdText = "BACKWARD"; break;
        case 'a': twist.angular.z = 1.0; cmdText = "TURN LEFT"; break;
        case 'd': twist.angular.z = -1.0; cmdText = "TURN RIGHT"; break;
        case ' ': twist.linear.x = 0; twist.angular.z = 0; cmdText = "STOP"; break;
        default: return;
    }

    // ROS 2 토픽 발행
    cmdVel.publish(twist);

    // 대시보드 UI 텍스트 업데이트
    const cmdDisp = document.getElementById('cmd-text');
    if (cmdDisp) {
        cmdDisp.innerText = cmdText;
        cmdDisp.style.color = (cmdText === "STOP") ? '#e74c3c' : '#2ecc71';
    }
}

// 명시적으로 전역 객체에 등록
window.sendCommand = sendCommand;

/**
 * 3. 페이지 로드 초기화 로직
 */
window.onload = function() {

    // --- ROSBridge 연결 이벤트 ---
    ros.on('connection', () => {
        console.log('ROSBridge 연결 성공!');
        const statusMsg = document.getElementById('status-msg');
        if (statusMsg) {
            statusMsg.innerText = 'Connected';
            statusMsg.style.color = '#2ecc71';
        }
        updateVideoStream();
    });

    ros.on('error', (error) => {
        console.error('ROSBridge 연결 에러:', error);
        const statusMsg = document.getElementById('status-msg');
        if (statusMsg) {
            statusMsg.innerText = 'Error';
            statusMsg.style.color = '#e74c3c';
        }
    });

    // --- 영상 스트리밍 설정 ---
    function updateVideoStream() {
        const videoElement = document.getElementById('remote-video');
        if (videoElement) {
            const topicName = '/camera/debug_image';
            // web_video_server(8080 포트)를 통해 MJPEG 스트림 수신
            videoElement.src = `http://${window.location.hostname}:8080/stream?topic=${topicName}`;
        }
    }

    // --- Chart.js 레이턴시 그래프 초기화 ---
    const canvas = document.getElementById('latencyChart');
    let latencyChart;
    if (canvas) {
        const ctx = canvas.getContext('2d');
        latencyChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Robot 1 Latency (ms)',
                    data: [],
                    borderColor: '#3498db',
                    backgroundColor: 'rgba(52, 152, 219, 0.1)',
                    borderWidth: 2,
                    pointRadius: 0,
                    fill: true,
                    tension: 0.3
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    x: { display: false },
                    y: { beginAtZero: true, grid: { color: '#333' } }
                },
                plugins: { legend: { display: false } }
            }
        });
    }

    // --- 로봇 1 레이턴시 데이터 구독 및 업데이트 ---
    latencyEcho.subscribe((message) => {
        const T2 = Date.now();
        const T1 = message.data;
        const rtt = T2 - T1;
        const oneWayLatency = (rtt / 2).toFixed(1);

        const textElement = document.getElementById('latency-text');
        if (textElement) textElement.innerText = oneWayLatency;

        if (latencyChart) {
            const timeStr = new Date().toLocaleTimeString();
            latencyChart.data.labels.push(timeStr);
            latencyChart.data.datasets[0].data.push(oneWayLatency);

            // 데이터가 30개를 넘으면 오래된 데이터 삭제
            if (latencyChart.data.labels.length > 30) {
                latencyChart.data.labels.shift();
                latencyChart.data.datasets[0].data.shift();
            }
            latencyChart.update('none');
        }
    });

    // --- 키보드 이벤트 핸들러 ---
    const keyToBtnId = {
        'w': 'btn-up', 'W': 'btn-up',
        'a': 'btn-left', 'A': 'btn-left',
        's': 'btn-down', 'S': 'btn-down',
        'd': 'btn-right', 'D': 'btn-right',
        ' ': 'btn-stop'
    };

    const keyState = {};

    window.addEventListener('keydown', (e) => {
        const key = e.key.toLowerCase();
        if (keyToBtnId[key] && !keyState[key]) {
            keyState[key] = true;
            const btn = document.getElementById(keyToBtnId[key]);
            if (btn) btn.classList.add('active');
            sendCommand(key);
        }
    });

    window.addEventListener('keyup', (e) => {
        const key = e.key.toLowerCase();
        if (keyToBtnId[key]) {
            keyState[key] = false;
            const btn = document.getElementById(keyToBtnId[key]);
            if (btn) btn.classList.remove('active');
        }
    });
};
