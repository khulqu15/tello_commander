% Parameter dan kondisi awal
nDrone = 5;
dMin = 50; % Jarak minimal untuk penghindaran tabrakan
vMax = 20; % Kecepatan maksimum drone
adjustHeight = 20; % Ketinggian penyesuaian untuk menghindari tabrakan
gravitasi = 9.81; % Percepatan gravitasi (m/s^2)
massa = 1.0; % Massa setiap drone dalam kilogram

k = 0.1; % Nilai ini bisa diubah untuk menyesuaikan responsivitas drone

alpha = k * (gravitasi / massa); % Perhitungan alpha dengan mempertimbangkan massa dan gravitasi
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
kecepatan_data = zeros(nDrone,3);
trayektori = zeros(1000, nDrone, 3); % Asumsi maksimal 1000 iterasi untuk penyimpanan
iterasi = 1;
allDronesAtTarget = false;

dt = 0.05;

while ~allDronesAtTarget && iterasi <= 100
    allDronesAtTarget = true; % Asumsikan semua drone di target hingga terbukti tidak
    for iterasiDrone = 1:nDrone
        % Hitung vektor menuju target
        vTarget = target_posisi(iterasiDrone,:) - posisi(iterasiDrone,:);
        % Menghitung akselerasi berdasarkan kecepatan target dan kecepatan saat ini
        a = (vTarget - kecepatan(iterasiDrone,:)) * alpha;
        
        % Update kecepatan berdasarkan akselerasi
        kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) + a * dt;
        kecepatan_data(iterasiDrone,:) = max(1, min(5, abs(kecepatan(iterasiDrone,:)))) .* sign(kecepatan(iterasiDrone,:));
        
        % Batasi kecepatan untuk tidak melebihi vMax
        if norm(kecepatan(iterasiDrone,:)) > vMax
            kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) / norm(kecepatan(iterasiDrone,:)) * vMax;
        end
        
        % Update posisi berdasarkan kecepatan baru
        posisi(iterasiDrone,:) = posisi(iterasiDrone,:) + kecepatan(iterasiDrone,:) * dt;

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
        kecepatan(iterasiDrone,:) = (alpha) * kecepatan(iterasiDrone,:) + alpha * vNew;
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

% Set up video writer dan figure untuk visualisasi
videoFile = 'drone_simulation_5_with_paths.avi';
v = VideoWriter(videoFile);

% Prediksi pertemuan drone
collisionTime = Inf; % Waktu sampai kolisi
collisionPoint = [Inf, Inf, Inf]; % Titik pertemuan

open(v);
figSimulasi = figure('Name', 'Simulasi Drone', 'Position', [100, 100, 1200, 600]);

kecepatanXHistory = zeros(1000, nDrone);
kecepatanYHistory = zeros(1000, nDrone);
kecepatanZHistory = zeros(1000, nDrone);

dataHistory = zeros(iterasi-1, nDrone*6);

% Loop utama untuk simulasi dan visualisasi
for iterasiFrame = 1:iterasi-1
    clf; % Bersihkan figure
    % Visualisasi Drone dan Trayektori

    for i = 1:nDrone
        % Menyimpan posisi dan kecepatan setiap drone ke dalam matriks
        dataHistory(iterasiFrame, (i-1)*6+1:(i-1)*6+6) = [posisi(i,:), kecepatan(i,:)];
    end

       
    subplot(4, nDrone, [1:nDrone*3]); % Subplot untuk simulasi drone
    hold on;
    view(3);
    grid on;
    axis equal;
    colors = lines(nDrone); % Menghasilkan warna berbeda untuk setiap drone
    for i = 1:nDrone-1
        for j = i+1:nDrone
            [t, p] = predictCollision(posisi(i,:), kecepatan(i,:), posisi(j,:), kecepatan(j,:), dt);
            if t < collisionTime
                collisionTime = t;
                collisionPoint = p;
            end
        end
    end

    % Gambar lintasan dan drone
    delete(findall(gcf,'type','annotation'));

    for i = 1:nDrone
        a = (target_posisi(i,:) - posisi(i,:)) * alpha; % Hitung akselerasi
        kecepatan(i,:) = kecepatan(i,:) + a * dt; % Perbarui kecepatan
        kecepatan(iterasiDrone,:) = max(1, min(5, abs(kecepatan(iterasiDrone,:)))) .* sign(kecepatan(iterasiDrone,:));
        posisi(i,:) = posisi(i,:) + kecepatan(i,:) * dt; % Perbarui posisi
        
        if norm(kecepatan(i,:)) > vMax
            kecepatan(i,:) = kecepatan(i,:) / norm(kecepatan(i,:)) * vMax;
        end
        plot3(posisi(i,1), posisi(i,2), posisi(i,3), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
        text(posisi(i,1), posisi(i,2), posisi(i,3) + adjustHeight, sprintf('Drone %d', i), 'FontSize', 10);
        
        quiver3(posisi(i,1), posisi(i,2), posisi(i,3), kecepatan(i,1), kecepatan(i,2), kecepatan(i,3), 'Color', colors(i,:));

        plot3([posisi_awal(i,1), target_posisi(i,1)], [posisi_awal(i,2), target_posisi(i,2)], [posisi_awal(i,3), target_posisi(i,3)], '--', 'Color', colors(i,:), 'LineWidth', 1);
        plot3(squeeze(trayektori(1:iterasiFrame,i,1)), squeeze(trayektori(1:iterasiFrame,i,2)), squeeze(trayektori(1:iterasiFrame,i,3)), 'Color', colors(i,:), 'LineWidth', 2);
        plot3(trayektori(iterasiFrame,i,1), trayektori(iterasiFrame,i,2), trayektori(iterasiFrame,i,3), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
        text(trayektori(iterasiFrame,i,1), trayektori(iterasiFrame,i,2), trayektori(iterasiFrame,i,3), sprintf('Drone %d', i), 'FontSize', 10, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
        quiver3(posisi(i,1), posisi(i,2), posisi(i,3), kecepatan(i,1), kecepatan(i,2), kecepatan(i,3), 'Color', colors(i,:), 'MaxHeadSize', 1, 'AutoScale', 'off');
    end

    % Jika ada prediksi tabrakan, tampilkan
    if collisionTime ~= Inf
        plot3(collisionPoint(1), collisionPoint(2), (collisionPoint(3) + 5), 'x', 'Color', 'red', 'MarkerSize', 10, 'LineWidth', 2);
        text(collisionPoint(1), (collisionPoint(2) + 30), (collisionPoint(3) + 60) + 10, sprintf('Crash in %.2f s', collisionTime), 'FontSize', 10, 'Color', 'red');
    end

    infoTextCells = arrayfun(@(idx) sprintf('Drone %d - Posisi: (%.2f, %.2f, %.2f), Kecepatan: (%.2f, %.2f, %.2f)', idx, posisi(idx,1), posisi(idx,2), posisi(idx,3), kecepatan(idx,1), kecepatan(idx,2), kecepatan(idx,3)), 1:nDrone, 'UniformOutput', false);
    infoText = strjoin(infoTextCells, '\n');
    annotation(gcf, 'textbox', [0, 0.85, 1, 0.15], 'String', infoText, 'FontSize', 7, 'EdgeColor', 'none', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    

    for i = 1:nDrone
        kecepatanXHistory(iterasiFrame, i) = kecepatan(i,1);
        kecepatanYHistory(iterasiFrame, i) = kecepatan(i,2);
        kecepatanZHistory(iterasiFrame, i) = kecepatan(i,3);

        % Plot kecepatan X
        subplot(4, nDrone, nDrone * 3 + i);
        plot(1:iterasiFrame, kecepatanXHistory(1:iterasiFrame, i), 'r', 'LineWidth', 2);
        hold on;
        % Plot kecepatan Y
        plot(1:iterasiFrame, kecepatanYHistory(1:iterasiFrame, i), 'g', 'LineWidth', 2);
        % Plot kecepatan Z
        plot(1:iterasiFrame, kecepatanZHistory(1:iterasiFrame, i), 'b', 'LineWidth', 2);
        hold off;

        xlim([1, iterasi]); % Sesuaikan batas x dengan jumlah iterasi simulasi
        ylim([-vMax, vMax]); % Batas y berdasarkan kecepatan maksimum
        title(sprintf('Kecepatan Drone %d (XYZ)', i));
        xlabel('Iterasi');
        ylabel('Kecepatan (unit/s)');
        legend('Vx', 'Vy', 'Vz');
    end
    drawnow; % Perbarui visualisasi

    % Capture dan simpan frame
    frame = getframe(figSimulasi);
    writeVideo(v, frame);
end

close(v); % Tutup file video setelah loop selesai
disp(['Video saved to ', videoFile]);

posisiKecepatanFilename = 'posisi_dan_kecepatan_history.csv';
csvwrite(posisiKecepatanFilename, dataHistory);
disp(['Data posisi dan kecepatan disimpan ke ', posisiKecepatanFilename]);


function [time, point] = predictCollision(pos1, vel1, pos2, vel2, dt)
    relativePosition = pos2 - pos1;
    relativeVelocity = vel2 - vel1;
    t = dot(-relativePosition, relativeVelocity) / (norm(relativeVelocity)^2);
    if t > 0 && t < dt % Periksa apakah tabrakan terjadi dalam langkah waktu ini
        point = pos1 + vel1 * t;
    else
        t = Inf;
        point = [Inf, Inf, Inf];
    end
    time = t;
end
