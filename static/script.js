window.onload = function() {
    // 1. ROS 연결 설정
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

    ros.on('connection', () => {
        console.log('ROSBridge 연결 성공!');
        const statusMsg = document.getElementById('status-msg');
        if (statusMsg) {
            statusMsg.innerText = 'Connected';
            statusMsg.style.color = 'var(--success-color)';
        }
    });

    ros.on('error', (error) => {
        console.error('ROSBridge 연결 에러:', error);
    });

    // 2. 토픽 설정
    const cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    const imageTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/image_raw',
        messageType: 'sensor_msgs/Image'
    });

    // [RTT 추가] 브리지 노드에서 돌아오는 에코 메시지 구독
    const latencyEcho = new ROSLIB.Topic({
        ros: ros,
        name: '/latency_echo',
        messageType: 'std_msgs/Float64'
    });

    // 3. 그래프 초기화 (Chart.js)
    const canvas = document.getElementById('latencyChart');
    let latencyChart;

    if (canvas) {
        const ctx = canvas.getContext('2d');
        Chart.defaults.color = '#888';
        latencyChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Control Latency (ms)',
                    data: [],
                    borderColor: '#e74c3c',
                    backgroundColor: 'rgba(231, 76, 60, 0.1)',
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

    // 4. [제어 레이턴시 계산] RTT 방식 적용
    // 브리지 노드가 보낸 에코(T1)를 받아 현재 시간(T2)과 비교
    latencyEcho.subscribe((message) => {
        const T2 = Date.now(); // 현재 도착 시각
        const T1 = message.data; // 웹에서 출발했던 시각

        const rtt = T2 - T1; // 왕복 지연 시간
        const oneWayLatency = (rtt / 2).toFixed(1); // 단방향 추정치

        // 텍스트 업데이트
        const textElement = document.getElementById('latency-text');
        if (textElement) {
            textElement.innerText = oneWayLatency;
        }

        // 그래프 업데이트
        if (latencyChart) {
            const timeStr = new Date().toLocaleTimeString();
            latencyChart.data.labels.push(timeStr);
            latencyChart.data.datasets[0].data.push(oneWayLatency);

            if (latencyChart.data.labels.length > 30) {
                latencyChart.data.labels.shift();
                latencyChart.data.datasets[0].data.shift();
            }
            latencyChart.update('none');
        }
    });

    // 5. 조종 및 키보드 이벤트 로직
    const keyToBtnId = {
        'w': 'btn-up', 'W': 'btn-up',
        'a': 'btn-left', 'A': 'btn-left',
        's': 'btn-down', 'S': 'btn-down',
        'd': 'btn-right', 'D': 'btn-right',
        ' ': 'btn-stop'
    };

    const keyState = {};

    window.addEventListener('keydown', (e) => {
        if (keyToBtnId[e.key] && !keyState[e.key]) {
            keyState[e.key] = true;
            const btn = document.getElementById(keyToBtnId[e.key]);
            if (btn) btn.classList.add('active');
            sendCommand(e.key.toLowerCase());
        }
    });

    window.addEventListener('keyup', (e) => {
        if (keyToBtnId[e.key]) {
            keyState[e.key] = false;
            const btn = document.getElementById(keyToBtnId[e.key]);
            if (btn) btn.classList.remove('active');
        }
    });

    function sendCommand(key) {
        let twist = new ROSLIB.Message({
            linear: {
                x: 0,
                y: Date.now(), // [T1 저장] 현재 브라우저의 타임스탬프
                z: 0
            },
            angular: { x: 0, y: 0, z: 0 }
        });

        let cmdText = "STOP";
        switch(key) {
            case 'w': twist.linear.x = 0.5; cmdText = "FORWARD"; break;
            case 's': twist.linear.x = -0.5; cmdText = "BACKWARD"; break;
            case 'a': twist.angular.z = 1.0; cmdText = "TURN LEFT"; break;
            case 'd': twist.angular.z = -1.0; cmdText = "TURN RIGHT"; break;
            case ' ': twist.linear.x = 0; twist.angular.z = 0; cmdText = "STOP"; break;
            default: return;
        }

        cmdVel.publish(twist);

        const cmdDisp = document.getElementById('cmd-text');
        if (cmdDisp) {
            cmdDisp.innerText = cmdText;
            cmdDisp.style.color = (cmdText === "STOP") ? 'var(--accent-color)' : 'var(--success-color)';
        }
    }

    window.sendCommand = sendCommand;
};
