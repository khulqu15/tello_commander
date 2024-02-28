% Parameter dan kondisi awal
nDrone = 5;
dMin = 50; % Jarak minimal untuk penghindaran tabrakan
vMax = 20; % Kecepatan maksimum drone
adjustHeight = 20; % Ketinggian penyesuaian untuk menghindari tabrakan
alpha = 0.1; % Faktor kelembaman
targetTolerance = 10.0; % Toleransi jarak ke target

% Inisialisasi posisi awal dan target posisi secara manual untuk demo
posisi_awal = zeros(nDrone, 3);
target_posisi = zeros(nDrone, 3);
for i = 1:nDrone
    if mod(i, 3) == 0
        posisi_awal(i, :) = [0, (i-1)*13.33, 150 + (i-1)*10];
        target_posisi(i, :) = [400, 200 - (i-1)*13.33, 150 + (i-1)*5];
    elseif mod(i, 3) == 1
        posisi_awal(i, :) = [400, (i-1)*13.33, 150 + (i-1)*10];
        target_posisi(i, :) = [0, 200 - (i-1)*13.33, 150 + (i-1)*5];
    else
        posisi_awal(i, :) = [200, (i-1)*13.33, 150 + (i-1)*10];
        target_posisi(i, :) = [200, 200 - (i-1)*13.33, 150 + (i-1)*5];
    end
end

% Inisialisasi posisi, kecepatan, dan trayektori
posisi = posisi_awal;
kecepatan = zeros(nDrone,3);
trayektori = zeros(1000, nDrone, 3); % Asumsi maksimal 1000 iterasi untuk penyimpanan
iterasi = 1;
allDronesAtTarget = false;

while ~allDronesAtTarget && iterasi <= 100
    allDronesAtTarget = true; % Asumsikan semua drone di target hingga terbukti tidak
    for iterasiDrone = 1:nDrone
        % Hitung vektor menuju target
        vTarget = target_posisi(iterasiDrone,:) - posisi(iterasiDrone,:);
        norm_vTarget = norm(vTarget(1:2)); % Norm horizontal
        if norm_vTarget > 0
            vTarget(1:2) = vTarget(1:2) / norm_vTarget * vMax;
        end
        
        % Inisialisasi penghindaran tabrakan
        vAvoid = zeros(1,3);
        for j = 1:nDrone
            if iterasiDrone ~= j
                jarak = norm(posisi(iterasiDrone,1:2) - posisi(j,1:2)); % Jarak horizontal
                jarakZ = abs(posisi(iterasiDrone,3) - posisi(j,3)); % Jarak vertikal
                if jarak < dMin && jarakZ < adjustHeight
                    if posisi(iterasiDrone,3) <= posisi(j,3)
                        vAvoid(3) = vAvoid(3) - vMax; % Turun
                    else
                        vAvoid(3) = vAvoid(3) + vMax; % Naik
                    end
                end
            end
        end
        
        % Gabungkan vektor target dengan penghindaran dan terapkan kelembaman
        vNew = vTarget + vAvoid;
        kecepatan(iterasiDrone,:) = (1-alpha) * kecepatan(iterasiDrone,:) + alpha * vNew;
        if norm(kecepatan(iterasiDrone,:)) > vMax
            kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) / norm(kecepatan(iterasiDrone,:)) * vMax;
        end
        
        % Update posisi
        posisi(iterasiDrone,:) = posisi(iterasiDrone,:) + kecepatan(iterasiDrone,:);
        trayektori(iterasi, iterasiDrone, :) = posisi(iterasiDrone,:);
        
        % Periksa apakah drone ini telah mencapai target
        if norm(posisi(iterasiDrone,:) - target_posisi(iterasiDrone,:)) >= targetTolerance
            allDronesAtTarget = false; % Jika drone ini belum di target, loop harus berlanjut
        end
    end
    iterasi = iterasi + 1;
end

% Visualisasi
figure;
hold on;
grid on;
axis equal;

colors = lines(nDrone); % Menghasilkan warna berbeda untuk setiap drone

for i = 1:nDrone
    plot3(squeeze(trayektori(:,i,1)), squeeze(trayektori(:,i,2)), squeeze(trayektori(:,i,3)), 'Color', colors(i,:), 'LineWidth', 2);
end

plot3(posisi_awal(:,1), posisi_awal(:,2), posisi_awal(:,3), 'ko', 'MarkerFaceColor', 'k');
plot3(target_posisi(:,1), target_posisi(:,2), target_posisi(:,3), 'go', 'MarkerFaceColor', 'g');

view(3);
legend(arrayfun(@(x) sprintf('Drone %d', x), 1:nDrone, 'UniformOutput', false), 'Location', 'Best');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trayektori Drone dengan Flocking Algorithm untuk Penghindaran Tabrakan');

% Set up video writer
videoFile = 'drone_simulation_5.avi';
v = VideoWriter(videoFile);
open(v);

% Visualisasi setup
figure;
hold on;
grid on;
axis equal;
colors = lines(nDrone); % Menghasilkan warna berbeda untuk setiap drone

% Loop untuk setiap iterasi dan gambar
for iterasiFrame = 1:iterasi-1
    clf; % Clear figure untuk frame baru
    hold on;
    grid on;
    axis equal;
    
    % Gambar trayektori dan posisi saat ini untuk setiap drone
    for i = 1:nDrone
        plot3(squeeze(trayektori(1:iterasiFrame,i,1)), squeeze(trayektori(1:iterasiFrame,i,2)), squeeze(trayektori(1:iterasiFrame,i,3)), 'Color', colors(i,:), 'LineWidth', 2);
        plot3(trayektori(iterasiFrame,i,1), trayektori(iterasiFrame,i,2), trayektori(iterasiFrame,i,3), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
    end
    
    plot3(posisi_awal(:,1), posisi_awal(:,2), posisi_awal(:,3), 'ko', 'MarkerFaceColor', 'k');
    plot3(target_posisi(:,1), target_posisi(:,2), target_posisi(:,3), 'go', 'MarkerFaceColor', 'g');
    
    legend(arrayfun(@(x) sprintf('Drone %d', x), 1:nDrone, 'UniformOutput', false), 'Location', 'Best');
    view(3);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title(sprintf('3D Trayektori Drone dengan Flocking Algorithm untuk Penghindaran Tabrakan (Iterasi %d)', iterasiFrame));
    
    % Capture the frame
    frame = getframe(gcf);
    writeVideo(v,frame);
end

% Close video file
close(v);

% Inform user
disp(['Video saved to ', videoFile]);
