% Inisialisasi Parameter Simulasi
nDrone = 15;
dMin = 50; % Jarak minimal untuk penghindaran tabrakan
vMax = 20; % Kecepatan maksimum
adjustHeight = 20; % Ketinggian penyesuaian untuk menghindari tabrakan
alpha = 0.1; % Faktor kelembaman
targetTolerance = 10.0; % Toleransi jarak ke target

% Inisialisasi posisi awal dan target posisi
posisi_awal = zeros(nDrone, 3);
target_posisi = zeros(nDrone, 3);

for i = 1:nDrone
    angle = 2 * pi * i / nDrone;
    posisi_awal(i, :) = [200 + 100 * cos(angle), 100 + 50 * sin(angle), 150];
    % Pastikan untuk menambahkan nilai Z yang sesuai
    target_posisi(i, :) = [200 - posisi_awal(i, 1) + 200, 200 - posisi_awal(i, 2) + 100, posisi_awal(i, 3)]; % Tambahkan nilai Z
end
posisi = posisi_awal;
kecepatan = zeros(nDrone, 3);
iterasi = 0;
maxIterasi = 1000;
allDronesAtTarget = false;

% Persiapan Video
videoFile = 'droneSimulation.mp4';
v = VideoWriter(videoFile, 'Motion JPEG AVI');
v.FrameRate = 20;
open(v);

fig = figure;
hold on;
axis([0 400 0 200 0 300]);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trayektori Drone');

colors = lines(nDrone);

while ~allDronesAtTarget && iterasi < maxIterasi
    allDronesAtTarget = true;
    cla;
    
    for i = 1:nDrone
        plot3(posisi(i, 1), posisi(i, 2), posisi(i, 3), 'o', 'MarkerSize', 6, 'MarkerFaceColor', colors(i, :));
    end
    legend(arrayfun(@(x) sprintf('Drone %d', x), 1:nDrone, 'UniformOutput', false), 'Location', 'northeastoutside');
    drawnow;
    
    frame = getframe(fig);
    writeVideo(v, frame);
    
    for i = 1:nDrone
        vTarget = (target_posisi(i,:) - posisi(i,:));
        norm_vTarget = norm(vTarget(1:2));
        if norm_vTarget > 0
            vTarget(1:2) = vTarget(1:2) / norm_vTarget * vMax;
        end

        vAvoid = zeros(1, 3);
        for j = 1:nDrone
            if i ~= j
                jarak = norm(posisi(i, 1:2) - posisi(j, 1:2));
                jarakZ = abs(posisi(i, 3) - posisi(j, 3));
                if jarak < dMin && jarakZ < adjustHeight
                    if posisi(i, 3) <= posisi(j, 3)
                        vAvoid(3) = vAvoid(3) - vMax;
                    else
                        vAvoid(3) = vAvoid(3) + vMax;
                    end
                end
            end
        end

        vNew = vTarget + vAvoid;
        kecepatan(i, :) = (1 - alpha) * kecepatan(i, :) + alpha * vNew;
        if norm(kecepatan(i, :)) > vMax
            kecepatan(i, :) = kecepatan(i, :) / norm(kecepatan(i, :)) * vMax;
        end

        posisi(i, :) = posisi(i, :) + kecepatan(i, :);
        if norm(posisi(i, :) - target_posisi(i, :)) >= targetTolerance
            allDronesAtTarget = false;
        end
    end
    
    iterasi = iterasi + 1;
end

close(v);
close(fig);
