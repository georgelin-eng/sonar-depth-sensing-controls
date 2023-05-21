clear

a = arduino ('COM8', 'Uno');
imu = mpu6050(a,'I2CAddress','0x68');

Accel_comps = zeros (100, 3);
Accel_norm = zeros (100, 1);

for n = 1:100 
    accel = readAcceleration (imu);
    Accel_comps (n ,:) = accel;
    Accel_norm (n) = norm(accel);
end