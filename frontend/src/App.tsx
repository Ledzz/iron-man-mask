import React, { useCallback, useEffect, useRef, useState } from "react";

let characteristic: BluetoothRemoteGATTCharacteristic | undefined;
const serviceUuid = "4fafc201-1fb5-459e-8fcc-c5c9c331914c";
const characteristicUuid = "beb5483e-36e1-4688-b7f5-ea07361b26a9";

const log = console.log;

const NUM_SERVOS = 8;
const NUM_POSES = 8;

interface ServoConfig {
  position: number; // 0-180 degrees
}

interface Pose {
  name: string;
  servos: ServoConfig[];
}

interface Config {
  poses: Pose[];
  currentPose: number;
}

const defaultPose = (): Pose => ({
  name: "New Pose",
  servos: Array(NUM_SERVOS).fill(null).map(() => ({ position: 90 }))
});

const defaultConfig = (): Config => ({
  poses: Array(NUM_POSES).fill(null).map((_, i) => ({
    ...defaultPose(),
    name: `Pose ${i + 1}`
  })),
  currentPose: 0
});

function ServoControlApp() {
  const [currentConfig, setCurrentConfig] = useState<Config>(defaultConfig());
  const [isConnected, setIsConnected] = useState(false);
  const [editingPose, setEditingPose] = useState<number | null>(null);

  const autoConnect = useCallback(async () => {
    try {
      log("Getting existing permitted Bluetooth devices...");
      const devices = await navigator.bluetooth.getDevices();

      log("> Got " + devices.length + " Bluetooth devices.");
      for (const device of devices) {
        connectToDevice(device);
      }
    } catch (error) {
      log("Argh! " + error);
    }
  }, []);

  const readConfiguration = useCallback(async () => {
    if (!characteristic) return;

    try {
      await characteristic.startNotifications();
      await sendCommand("GET_CONFIG");

      characteristic.addEventListener("characteristicvaluechanged", (event) => {
        const value = (event.target as BluetoothRemoteGATTCharacteristic).value;
        if (value) {
          const decoder = new TextDecoder();
          const configString = decoder.decode(value);
          try {
            const config = JSON.parse(configString);
            setCurrentConfig(config);
            log("Received configuration:", config);
          } catch (e) {
            log("Error parsing configuration:", e);
          }
        }
      });
    } catch (error) {
      log("Error reading configuration:", error);
    }
  }, []);

  const connectToDevice = useCallback(async (device: BluetoothDevice) => {
    const abortController = new AbortController();

    device.addEventListener(
        "advertisementreceived",
        async (event: BluetoothAdvertisingEvent) => {
          log('> Received advertisement from "' + device.name + '"...');
          abortController.abort();
          log('Connecting to GATT Server from "' + device.name + '"...');
          try {
            const server = await device.gatt?.connect();
            log('> Bluetooth device "' + device.name + " connected.");

            const service = await server?.getPrimaryService(serviceUuid);
            characteristic = await service?.getCharacteristic(characteristicUuid);
            setIsConnected(true);

            await readConfiguration();
          } catch (error) {
            log("Argh! " + error);
            setIsConnected(false);
          }
        },
        { once: true },
    );

    try {
      log('Watching advertisements from "' + device.name + '"...');
      await device.watchAdvertisements({ signal: abortController.signal });
    } catch (error) {
      log("Argh! " + error);
    }
  }, [readConfiguration]);

  const manualConnect = useCallback(async () => {
    try {
      log("Requesting any Bluetooth device...");
      const device = await navigator.bluetooth.requestDevice({
        filters: [
          {
            services: [serviceUuid],
          },
        ],
      });

      log("> Requested " + device.name);
      await connectToDevice(device);
    } catch (error) {
      log("Argh! " + error);
    }
  }, [connectToDevice]);

  const startedRef = useRef(false);

  useEffect(() => {
    if (!startedRef.current) {
      // Load config from memory state (would be localStorage in real app)
      autoConnect();
    }
    startedRef.current = true;
  }, [autoConnect]);

  // Save config to memory state whenever it changes
  useEffect(() => {
    // In a real implementation with localStorage, you'd do:
    // localStorage.setItem('servoConfig', JSON.stringify(currentConfig));
    log("Config updated:", currentConfig);
  }, [currentConfig]);

  const sendCommand = useCallback(async (command: string) => {
    console.log("Send Command:", command);
    if (characteristic) {
      const encoder = new TextEncoder();
      await characteristic.writeValue(encoder.encode(command));
    }
  }, []);

  const sendPoseCommand = useCallback((poseIndex: number) => {
    const pose = currentConfig.poses[poseIndex];
    if (!pose) return;

    // Format: POSE:poseIndex:servo0:servo1:...:servo7
    const positions = pose.servos.map(s => s.position.toString()).join(':');
    const command = `POSE:${poseIndex}:${positions}`;

    setCurrentConfig(prev => ({ ...prev, currentPose: poseIndex }));
    return sendCommand(command);
  }, [currentConfig.poses, sendCommand]);

  const sendServoCommand = useCallback((servoIndex: number, position: number) => {
    // Format: SERVO:servoIndex:position
    const command = `SERVO:${servoIndex}:${position}`;
    return sendCommand(command);
  }, [sendCommand]);

  const updateServoPosition = (poseIndex: number, servoIndex: number, position: number) => {
    setCurrentConfig(prev => ({
      ...prev,
      poses: prev.poses.map((pose, pi) =>
          pi === poseIndex
              ? {
                ...pose,
                servos: pose.servos.map((servo, si) =>
                    si === servoIndex ? { ...servo, position } : servo
                )
              }
              : pose
      )
    }));

    // Send individual servo command for real-time control
    if (poseIndex === currentConfig.currentPose) {
      sendServoCommand(servoIndex, position);
    }
  };

  const updatePoseName = (poseIndex: number, name: string) => {
    setCurrentConfig(prev => ({
      ...prev,
      poses: prev.poses.map((pose, pi) =>
          pi === poseIndex ? { ...pose, name } : pose
      )
    }));
  };

  const copyPose = (sourcePoseIndex: number, targetPoseIndex: number) => {
    const sourcePose = currentConfig.poses[sourcePoseIndex];
    if (!sourcePose) return;

    setCurrentConfig(prev => ({
      ...prev,
      poses: prev.poses.map((pose, pi) =>
          pi === targetPoseIndex
              ? {
                ...sourcePose,
                name: `${sourcePose.name} Copy`
              }
              : pose
      )
    }));
  };

  return (
      <div className="min-h-screen bg-gray-900 text-white p-6">
        <div className="max-w-6xl mx-auto">
          <h1 className="text-3xl font-bold mb-6">Servo Control Interface</h1>

          {/* Connection Status */}
          <div className="mb-6 p-4 bg-gray-800 rounded-lg">
            <div className="flex items-center gap-4">
              <div className={`px-3 py-1 rounded text-sm font-medium ${
                  isConnected ? 'bg-green-600' : 'bg-red-600'
              }`}>
                {isConnected ? 'Connected' : 'Not Connected'}
              </div>
              {!isConnected && (
                  <button
                      onClick={manualConnect}
                      className="px-4 py-2 bg-blue-600 hover:bg-blue-700 rounded transition-colors"
                  >
                    Connect via Bluetooth
                  </button>
              )}
            </div>
          </div>

          {/* Quick Pose Selection */}
          <div className="mb-8 p-4 bg-gray-800 rounded-lg">
            <h2 className="text-xl font-semibold mb-4">Quick Pose Selection</h2>
            <div className="grid grid-cols-4 gap-3">
              {currentConfig.poses.map((pose, index) => (
                  <button
                      key={index}
                      onClick={() => sendPoseCommand(index)}
                      disabled={!isConnected}
                      className={`p-3 rounded transition-colors ${
                          !isConnected
                              ? 'bg-gray-600 text-gray-400 cursor-not-allowed'
                              : currentConfig.currentPose === index
                                  ? 'bg-blue-600 hover:bg-blue-700'
                                  : 'bg-gray-700 hover:bg-gray-600'
                      }`}
                  >
                    <div className="font-medium">{pose.name}</div>
                    <div className="text-sm text-gray-300">Pose {index + 1}</div>
                  </button>
              ))}
            </div>
          </div>

          {/* Live Servo Control */}
          {isConnected && (
              <div className="mb-8 p-4 bg-gray-800 rounded-lg">
                <h2 className="text-xl font-semibold mb-4">Live Servo Control</h2>
                <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                  {currentConfig.poses[currentConfig.currentPose]?.servos.map((servo, servoIndex) => (
                      <div key={servoIndex} className="space-y-2">
                        <label className="block text-sm font-medium">
                          Servo {servoIndex + 1}
                        </label>
                        <input
                            type="range"
                            min="0"
                            max="180"
                            value={servo.position}
                            onChange={(e) => updateServoPosition(currentConfig.currentPose, servoIndex, parseInt(e.target.value))}
                            className="w-full"
                        />
                        <div className="flex items-center gap-2">
                          <input
                              type="number"
                              min="0"
                              max="180"
                              value={servo.position}
                              onChange={(e) => updateServoPosition(currentConfig.currentPose, servoIndex, parseInt(e.target.value) || 0)}
                              className="w-16 px-2 py-1 bg-gray-700 rounded border border-gray-600 text-white text-sm"
                          />
                          <span className="text-sm text-gray-300">°</span>
                        </div>
                      </div>
                  ))}
                </div>
              </div>
          )}

          {/* Pose Editor */}
          <div className="space-y-6">
            <h2 className="text-xl font-semibold">Pose Editor</h2>
            {currentConfig.poses.map((pose, poseIndex) => (
                <div key={poseIndex} className="p-4 bg-gray-800 rounded-lg">
                  <div className="flex items-center gap-4 mb-4">
                    <input
                        type="text"
                        value={pose.name}
                        onChange={(e) => updatePoseName(poseIndex, e.target.value)}
                        className="px-3 py-2 bg-gray-700 rounded border border-gray-600 text-white"
                        placeholder="Pose name"
                    />
                    <button
                        onClick={() => sendPoseCommand(poseIndex)}
                        disabled={!isConnected}
                        className={`px-4 py-2 rounded transition-colors ${
                            !isConnected
                                ? 'bg-gray-600 text-gray-400 cursor-not-allowed'
                                : 'bg-green-600 hover:bg-green-700'
                        }`}
                    >
                      Apply Pose
                    </button>
                    <button
                        onClick={() => setEditingPose(editingPose === poseIndex ? null : poseIndex)}
                        className="px-4 py-2 bg-gray-600 hover:bg-gray-700 rounded transition-colors"
                    >
                      {editingPose === poseIndex ? 'Hide' : 'Edit'}
                    </button>
                  </div>

                  {editingPose === poseIndex && (
                      <>
                        <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-4">
                          {pose.servos.map((servo, servoIndex) => (
                              <div key={servoIndex} className="space-y-2">
                                <label className="block text-sm font-medium">
                                  Servo {servoIndex + 1}
                                </label>
                                <input
                                    type="range"
                                    min="0"
                                    max="180"
                                    value={servo.position}
                                    onChange={(e) => updateServoPosition(poseIndex, servoIndex, parseInt(e.target.value))}
                                    className="w-full"
                                />
                                <div className="flex items-center gap-2">
                                  <input
                                      type="number"
                                      min="0"
                                      max="180"
                                      value={servo.position}
                                      onChange={(e) => updateServoPosition(poseIndex, servoIndex, parseInt(e.target.value) || 0)}
                                      className="w-16 px-2 py-1 bg-gray-700 rounded border border-gray-600 text-white text-sm"
                                  />
                                  <span className="text-sm text-gray-300">°</span>
                                </div>
                              </div>
                          ))}
                        </div>

                        <div className="flex gap-2">
                          <select
                              onChange={(e) => {
                                const sourceIndex = parseInt(e.target.value);
                                if (sourceIndex !== poseIndex) {
                                  copyPose(sourceIndex, poseIndex);
                                }
                              }}
                              className="px-3 py-2 bg-gray-700 rounded border border-gray-600 text-white"
                              defaultValue=""
                          >
                            <option value="" disabled>Copy from...</option>
                            {currentConfig.poses.map((p, i) => (
                                <option key={i} value={i} disabled={i === poseIndex}>
                                  {p.name}
                                </option>
                            ))}
                          </select>
                        </div>
                      </>
                  )}
                </div>
            ))}
          </div>

          {/* Debug Console */}
          <div className="mt-8 p-4 bg-gray-800 rounded-lg">
            <h3 className="text-lg font-semibold mb-2">Debug Info</h3>
            <div className="text-sm text-gray-300">
              <p>Current Pose: {currentConfig.currentPose + 1} ({currentConfig.poses[currentConfig.currentPose]?.name})</p>
              <p>Connection Status: {isConnected ? 'Connected' : 'Disconnected'}</p>
              <p>Device: Servo Mask</p>
            </div>
          </div>
        </div>
      </div>
  );
}

export default ServoControlApp;