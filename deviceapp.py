import asyncio
import json
import imu
from time import time
from datetime import datetime
from math import sin, cos, tan, pi, sqrt


phi_hat = 0.0
theta_hat = 0.0
attitudes_dict = {}
calibrated = False


async def est_attitude():
    global phi_hat, theta_hat
    mpu = imu.IMU()
    loop = asyncio.get_running_loop()

    alpha = 0.1
    dt = 0.0
    start_time = time()

    while True:
        try:
            dt = time() - start_time
            start_time = time()

            [phi_hat_acc, theta_hat_acc] = await loop.run_in_executor(None, mpu.get_acc_angles)

            [p, q, r] = await loop.run_in_executor(None, mpu.get_gyro)
            p -= mpu.gyro_bias[0]
            q -= mpu.gyro_bias[1]
            r -= mpu.gyro_bias[2]

            phi_dot = p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * r
            theta_dot = cos(phi_hat) * q - sin(phi_hat) * r

            phi_hat = (1 - alpha) * (phi_hat + dt * phi_dot) + alpha * phi_hat_acc
            theta_hat = (1 - alpha) * (theta_hat + dt * theta_dot) + alpha * theta_hat_acc

        except RuntimeError:
            pass

        await asyncio.sleep(0.02)


def predict_attitude(phi, theta):
    global attitudes_dict
    euc_dist = {}
    for k, v in attitudes_dict.items():
        euc_dist[k] = sqrt((phi - attitudes_dict[k][0])**2 + (theta - attitudes_dict[k][1])**2)
    return min(euc_dist, key=euc_dist.get)


async def comm_attitude():
    global phi_hat, theta_hat, calibrated
    loop = asyncio.get_running_loop()
    current_att = 0
    while True:
        if calibrated:
            predicted_att = predict_attitude(phi_hat, theta_hat)
            if predicted_att != current_att:
                current_att = predicted_att
                try:
                    with open('/dev/rfcomm0', 'w', 1) as serdev:
                        ctime = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                        await loop.run_in_executor(None, lambda: serdev.write(', '.join([ctime, predicted_att])))
                        await loop.run_in_executor(None, lambda: serdev.write('\n'))
                except PermissionError:
                    print('Connection not ready, retrying...')
        else:
            pass

        await asyncio.sleep(1)


async def calibrate():
    global phi_hat, theta_hat, attitudes_dict, calibrated
    cal = input('Calibrate? [y/N] ')
    if cal == 'y':
        for pos in range(1, 6):
            input('Rotate to position ' + str(pos))
            for i in range(100):
                await asyncio.sleep(.1)
            attitudes_dict[pos] = [phi_hat, theta_hat]

        print('Calibration completed: ')
        print(attitudes_dict)
        calibrated = True
        with open('attitudes.json', 'w') as f:
            json.dump(attitudes_dict, f)
    else:
        try:
            with open('attitudes.json', 'r') as f:
                attitudes_dict = json.load(f)
                calibrated = True
        except:
            calibrated = False


async def main():
    await asyncio.gather(est_attitude(), calibrate(), comm_attitude())


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('Bye.')
