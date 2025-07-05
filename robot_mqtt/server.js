require('dotenv').config(); // .env 파일 로드
const mqtt = require('mqtt');
const { exec, spawn } = require('child_process'); // child_process 모듈 다시 추가, spawn 추가

// 환경 변수에서 접속 정보 로드
const brokerUrl = process.env.MQTT_BROKER_URL;
const port = process.env.MQTT_PORT;
const username = process.env.MQTT_USERNAME;
const password = process.env.MQTT_PASSWORD;

if (!brokerUrl || !port || !username || !password) {
    console.error('Error: MQTT connection details missing in .env file.');
    process.exit(1); // 필수 정보 없으면 종료
}

const options = {
    port: parseInt(port, 10), // 포트 번호를 숫자로 변환
    username: username,
    password: password,
    clientId: `robot_publisher_${Math.random().toString(16).substr(2, 8)}`, // 고유 클라이언트 ID
    // 추가 옵션 (필요시): clean: true, connectTimeout: 4000, reconnectPeriod: 1000 등
};

console.log(`Attempting to connect to MQTT broker at ${brokerUrl}:${port}`);

// MQTT 브로커에 연결
const client = mqtt.connect(brokerUrl, options);

// --- State variable for arrival message ---
// 도착 메시지가 이미 전송되었는지 추적하는 플래그
let isArrivalMessageSent = false;
// -----------------------------------------

// 연결 성공 시
client.on('connect', function () {
    console.log('Successfully connected to MQTT broker');

    // QR 스캔 완료 토픽 구독
    const scanCompleteTopic = 'scan/complete';
    client.subscribe(scanCompleteTopic, { qos: 0 }, (error) => {
        if (error) {
            console.error(`Failed to subscribe to topic ${scanCompleteTopic}:`, error);
        } else {
            console.log(`Successfully subscribed to topic: ${scanCompleteTopic}`);
        }
    });

    // 네비게이션 시작 토픽 구독 추가
    const navigationStartTopic = 'navigation/start';
    client.subscribe(navigationStartTopic, { qos: 1 }, (error) => { // QoS 1로 설정
        if (error) {
            console.error(`Failed to subscribe to topic ${navigationStartTopic}:`, error);
        } else {
            console.log(`Successfully subscribed to topic: ${navigationStartTopic}`);
        }
    });

    // 네비게이션 컨트롤 토픽 구독 추가
    const navigationControlTopic = 'navigation/control';
    client.subscribe(navigationControlTopic, { qos: 1 }, (error) => {
        if (error) {
            console.error(`Failed to subscribe to topic ${navigationControlTopic}:`, error);
        } else {
            console.log(`Successfully subscribed to topic: ${navigationControlTopic}`);
        }
    });

    // --- /web 토픽 처리 로직 수정 ---
    const goalReachedMqttTopic = 'navigation/web';
    const rosSetupPath = '/opt/ros/humble/setup.bash';
    const webTopicType = 'std_msgs/msg/String'; // '/web' 토픽의 실제 메시지 타입으로 변경하세요.
    const rosEchoCommand = `source ${rosSetupPath} && ros2 topic echo /web ${webTopicType}`;

    console.log(`Attempting to listen to ROS topic /web using command: bash -c "${rosEchoCommand}"`);

    // ROS 리스너를 시작하는 함수 (재시작을 위해)
    function startRosListener() {
        // 리스너 시작 전 플래그 초기화 (선택적: 리스너 재시작 시 초기화할 경우)
        // isArrivalMessageSent = false;

        // --- 추가: 리스너 에러 발생 여부 추적 플래그 ---
        let listenerHasError = false;

        const rosListener = spawn('bash', ['-c', rosEchoCommand], { stdio: ['ignore', 'pipe', 'pipe'] });

        rosListener.stdout.on('data', (data) => {
            // --- 수정: 실제 도착 메시지 확인 로직 제거 ---
            const messageContent = data.toString().trim();
            console.log(`Received data from /web echo: ${messageContent}`); // 수신 데이터 로그

            // --- 수정: isActualArrivalMessage 조건 제거 ---
            if (!listenerHasError && !isArrivalMessageSent) {
                console.log('Data received from /web echo. Publishing arrival status to MQTT...'); // 로그 메시지 수정
                const arrivalMessage = JSON.stringify({ status: 'arrived' });
                client.publish(goalReachedMqttTopic, arrivalMessage, { qos: 1 }, (error) => {
                    if (error) {
                        console.error(`Failed to publish arrival status to ${goalReachedMqttTopic}:`, error);
                        // 발행 실패 시 플래그를 다시 false로 설정하여 재시도 가능하게 할 수 있음 (선택적)
                        // isArrivalMessageSent = false;
                    } else {
                        console.log(`Published arrival status to ${goalReachedMqttTopic}: ${arrivalMessage}`);
                        // 성공적으로 발행했으므로 플래그를 true로 설정
                        isArrivalMessageSent = true;
                        console.log('Arrival message sent flag set to true.');
                        // 도착 메시지 발행 후 리스너 종료 (선택적)
                        // rosListener.kill();
                        // console.log('ROS topic listener stopped after sending arrival message.');
                    }
                });
            } else if (isArrivalMessageSent) {
                console.log('Arrival message already sent. Ignoring subsequent echo output.');
            } else if (listenerHasError) {
                console.log('Listener encountered an error. Ignoring echo output.');
            }
        });

        rosListener.stderr.on('data', (data) => {
            const stderrStr = data.toString().trim();
            if (stderrStr && !stderrStr.includes('waiting for message')) {
                 console.error(`ROS topic listener stderr: ${stderrStr}`);
                 // --- 추가: 특정 에러 발생 시 플래그 설정 ---
                 if (stderrStr.includes('Could not determine the type')) {
                     listenerHasError = true;
                     console.error('Critical error detected: Could not determine topic type. Arrival message will not be sent.');
                 }
            }
        });

        rosListener.on('close', (code) => {
            console.log(`ROS topic listener process exited with code ${code}`);
            if (code !== 0) {
                 console.error(`ROS listener exited unexpectedly (code: ${code}).`);
                 listenerHasError = true; // --- 추가: 비정상 종료 시 에러 플래그 설정 ---
                 // 예시: 5초 후 리스너 재시작 시도 (주의: 무한 재시작 루프 방지 로직 필요)
                 // setTimeout(startRosListener, 5000);
            }
             // 정상 종료 또는 재시작 시 플래그 초기화 필요 여부 결정
             // isArrivalMessageSent = false; // 필요 시 주석 해제
        });

        rosListener.on('error', (err) => {
            console.error('Failed to start ROS topic listener process:', err);
            listenerHasError = true; // --- 추가: 프로세스 시작 실패 시 에러 플래그 설정 ---
            // 에러 발생 시에도 재시작 로직 고려 가능
            // setTimeout(startRosListener, 5000);
        });
    }

    // 초기 ROS 리스너 시작
    startRosListener();
    // --- /web 토픽 처리 로직 끝 ---
});

// 메시지 수신 시 처리
client.on('message', (topic, message) => {
    const messageString = message.toString();
    console.log(`Received message from topic ${topic}: ${messageString}`);

    // scan/complete 토픽 처리 (기존 로직)
    if (topic === 'scan/complete') {
        try {
            const receivedData = JSON.parse(messageString);
            console.log('Scan Complete Topic Received:', receivedData);
            // 기존 로직 수행 가능 (예: receivedData.message === 'hi!' 인지 확인 등)
            // 이 예제에서는 scan/complete 토픽에 대한 ROS 명령어 실행은 제거했습니다.
            // 만약 scan/complete 토픽에서도 ROS 명령어가 필요하다면 해당 로직을 여기에 다시 추가해야 합니다.
        } catch (e) {
            console.error('Failed to parse message JSON for scan/complete:', e);
        }
    }
    // navigation/start 토픽 처리 (새 로직)
    else if (topic === 'navigation/start') {
        try {
            const receivedData = JSON.parse(messageString);
            console.log('Navigation Start Topic Received:', receivedData);

            if (receivedData && receivedData.command === 'start_navigation') {
                console.log('Received start_navigation command. Executing ROS 2 service call...');

                // --- 새로운 네비게이션 시작 시 도착 플래그 초기화 ---
                console.log('Resetting arrival message sent flag for new navigation.');
                isArrivalMessageSent = false;
                // --------------------------------------------------
                
                // num 값 추출 및 확인
                const numValue = receivedData.facilityNum || 1; // 기본값 1 설정
                console.log(`Using facility number: ${numValue} for service call`);

                // ROS 서비스 호출 명령어 실행
                const rosSetupPath = '/opt/ros/humble/setup.bash';
                const serviceCallCommand = `bash -c "source ${rosSetupPath} && ros2 topic pub --once /trigger_goal std_msgs/msg/Int64 '{data: ${numValue}}'"`;

                console.log(`Executing: ${serviceCallCommand}`);
                exec(serviceCallCommand, (error, stdout, stderr) => {
                    if (error) {
                        console.error(`Error executing ROS 2 service call: ${error.message}`);
                        console.error(`stderr: ${stderr}`);
                        return;
                    }
                    if (stderr) {
                        console.log(`Service call stderr (might contain response): ${stderr}`);
                    }
                    console.log(`Service call stdout: ${stdout}`);
                    console.log('ROS 2 service call executed successfully.');
                });

                // 중요: 만약 startRosListener() 내부에서 리스너를 kill() 했다면,
                //      여기서 다시 startRosListener()를 호출하여 /web 토픽 감시를 재개해야 할 수 있습니다.
                // if (rosListener is not running) { startRosListener(); }

            } else {
                console.log('Received message on navigation/start topic, but not the start_navigation command.');
            }
        } catch (e) {
            console.error('Failed to parse message JSON or process command for navigation/start:', e);
        }
    }
    // navigation/control 토픽 처리
    else if (topic === 'navigation/control') {
        try {
            const receivedData = JSON.parse(messageString);
            console.log('Navigation Control Topic Received:', receivedData);

            if (receivedData && receivedData.command === 'return_to_base') {
                console.log('Received return_to_base command. Executing ROS 2 service call...');
                
                // ROS 서비스 호출 명령어 실행
                const rosSetupPath = '/opt/ros/humble/setup.bash';
                const serviceCallCommand = `bash -c "source ${rosSetupPath} && ros2 topic pub --once /trigger_goal std_msgs/msg/Int64 '{data: 0}'"`;

                console.log(`Executing: ${serviceCallCommand}`);
                exec(serviceCallCommand, (error, stdout, stderr) => {
                    if (error) {
                        console.error(`Error executing ROS 2 service call: ${error.message}`);
                        console.error(`stderr: ${stderr}`);
                        return;
                    }
                    if (stderr) {
                        console.log(`Service call stderr (might contain response): ${stderr}`);
                    }
                    console.log(`Service call stdout: ${stdout}`);
                    console.log('ROS 2 service call for return_to_base executed successfully.');
                });
            } else {
                console.log('Received message on navigation/control topic, but not recognized command:', receivedData.command);
            }
        } catch (e) {
            console.error('Failed to parse message JSON or process command for navigation/control:', e);
        }
    }
});

// 연결 에러 발생 시
client.on('error', function (error) {
    console.error('MQTT Connection Error:', error);
    // 에러 발생 시 재연결 시도 등 로직 추가 가능
});

// 연결 종료 시
client.on('close', function () {
    console.log('MQTT connection closed');
});

// 재연결 시도 시
client.on('reconnect', function () {
    console.log('Attempting to reconnect to MQTT broker...');
});

// 오프라인 시 (연결 끊김 감지)
client.on('offline', function () {
    console.log('MQTT client is offline');
}); 

