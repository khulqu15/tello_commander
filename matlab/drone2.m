nDrone = 5; % Jumlah Drone
dMin = 50; % Minimal Jarak Toleransi
vMax = 20; % Kecematan Maksimal
adjustHeight = 20; % Ketinggian 
gravitasi = 9.81; % Gravitasi
massa = 1.0; % Massa drone
k = 0.5; % Koefisien Percepatan

alpha = k * (gravitasi / massa); % 0.5 x (9.81 / 1.0) = 4.905
targetTolerance = 5; % Nilai toleransi agar drone landing

jarakAsli = zeros(1, nDrone); % Insiasi Jarak [0, 0, 0, 0, 0]
jarakMenghindari = zeros(1, nDrone); % Inisiasi Jarak Penghindaran [0, 0, 0, 0, 0]
collisionAvoidanceCounter = zeros(1, nDrone);
magnitudoKecepatanHistory = zeros(1000, nDrone);
waktuSampaiTujuan = zeros(1, nDrone); % Inisiasi Waktu sampai [0, 0, 0, 0, 0]

jarakMulaiMenghindari = Inf * ones(1, nDrone); % [Inf, Inf, Inf, Inf, Inf]

posisi_awal = zeros(nDrone, 3); % Inisiasi posisi awal [x,y,z], [x,y,z] ... 5
target_posisi = zeros(nDrone, 3); % Inisiasi posisi target [x,y,z], [x,y,z] ... 5
for i = 1:nDrone
    jarakAsli(i) = norm(target_posisi(i,:) - posisi_awal(i,:));
    if mod(i, 3) == 0
        posisi_awal(i, :) = [10, (i-1)*12, 5 + (i-1)*5];
        target_posisi(i, :) = [400, 200 - (i-1)*10, 150 + (i-1)*5];
    elseif mod(i, 3) == 1
        posisi_awal(i, :) = [400, (i-1)*12, 5 + (i-1)*5];
        target_posisi(i, :) = [0, 200 - (i-1)*10, 150 + (i-1)*5];
    else
        posisi_awal(i, :) = [200, (i-1)*12, 5 + (i-1)*5];
        target_posisi(i, :) = [200, 200 - (i-1)*10, 150 + (i-1)*5];
    end
end

posisi = posisi_awal;
kecepatan = zeros(nDrone,3);
trayektori = zeros(1000, nDrone, 3);
iterasi = 1;
allDronesAtTarget = false;

dt = 0.05;
distanceTrajectory = zeros(1, nDrone);
ketinggianTakeoff = 50;
ketinggianTarget = 150;
collisionAvoidanceStart = Inf * ones(1, nDrone);
collisionAvoidancePoint = Inf * ones(nDrone, 3);
jarakTotalPathAsli = zeros(1, nDrone);
jarakTotalPathNormal = zeros(1, nDrone);
landedDrone = zeros(1, nDrone);
takeoffRate = 5;
landingRate = 500;
decelerationRate = 0.5;

landingQueue = 1:nDrone;
currentlyLanding = 0;

priorities = [5, 3, 2, 1, 4];
landingPriorityQueue = zeros(1, nDrone);
ETA = zeros(1, nDrone);
%%

while ~allDronesAtTarget
    allDronesAtTarget = true;
    fprintf("Iterasi ke %d", iterasi);
    
    for i = 1:nDrone
        if landedDrone(i) == 0
            jarakKeTargetHorizontal = norm(target_posisi(i,1:2) - posisi(i,1:2));
            if norm(kecepatan(i,:)) > 0
                ETA(i) = jarakKeTargetHorizontal / norm(kecepatan(i,:));
            else
                ETA(i) = Inf;
            end
            priorities(i) = 1 / (jarakKeTargetHorizontal + 1) + 1 / (ETA(i) + 1);
        end
    end
    [~, priorityIndices] = sort(priorities, 'descend');
    %%

    for iterasiDrone = 1:nDrone
        jarakKeTargetHorizontal = norm(target_posisi(iterasiDrone,1:2) - posisi(iterasiDrone,1:2));
        jarakKeTargetVertical = abs(target_posisi(iterasiDrone,3) - posisi(iterasiDrone,3));

        vTarget = target_posisi(iterasiDrone,:) - posisi(iterasiDrone,:);
        a = (vTarget - kecepatan(iterasiDrone,:)) * alpha;
        
        kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) + a * dt;
        if norm(kecepatan(iterasiDrone,:)) > vMax
            kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) / norm(kecepatan(iterasiDrone,:)) * vMax;
        elseif jarakKeTargetHorizontal <= 0
            kecepatan(iterasiDrone,:) = max(kecepatan(iterasiDrone,:) - decelerationRate * dt, 0);
        end

        if norm(kecepatan(iterasiDrone,:)) < vMax && jarakKeTargetHorizontal > targetTolerance
            kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) + ((target_posisi(iterasiDrone,:) - posisi(iterasiDrone,:)) / norm(target_posisi(iterasiDrone,:) - posisi(iterasiDrone,:))) * alpha * dt;
        end

        if jarakKeTargetHorizontal <= targetTolerance && norm(kecepatan(iterasiDrone,:)) > 0
            kecepatanReductionFactor = max(0, 1 - decelerationRate * dt); % Faktor pengurangan kecepatan
            kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) * kecepatanReductionFactor;
        end

        vAvoid = zeros(1,3);
        for j = 1:nDrone
            if iterasiDrone ~= j && priorities(iterasiDrone) >= priorities(j)
                jarak = norm(posisi(iterasiDrone,1:2) - posisi(j,1:2));
                if jarak < 2
                    collisionAvoidanceCounter(j) = collisionAvoidanceCounter(j) + 1;
                end
                jarakZ = abs(posisi(iterasiDrone,3) - posisi(j,3));
                if jarak < dMin && jarakZ < adjustHeight
                    if iterasi < collisionAvoidanceStart(iterasiDrone)
                        collisionAvoidanceStart(iterasiDrone) = iterasi;
                        collisionAvoidancePoint(iterasiDrone, :) = posisi(iterasiDrone, :);
                        jarakMulaiMenghindari(iterasiDrone) = jarak;
                    end
                    if posisi(iterasiDrone,3) > posisi(j,3)
                        direction = (posisi(j,:) - posisi(iterasiDrone,:)) / norm(posisi(j,:) - posisi(iterasiDrone,:));
                        inertiaFactor = massa / (1 + alpha);
                        vAvoid = vAvoid + direction * vMax * inertiaFactor;
                    end
                    direction = (posisi(j,:) - posisi(iterasiDrone,:)) / norm(posisi(j,:) - posisi(iterasiDrone,:));
                    inertiaFactor = massa / (1 + alpha);
                    vAvoid = vAvoid + direction * vMax * inertiaFactor;
                end
            end
        end
        
        vNew = vTarget + vAvoid;
        kecepatan(iterasiDrone,:) = vNew * alpha;
        
        if landedDrone(iterasiDrone) == 0
            posisi(iterasiDrone,:) = posisi(iterasiDrone,:) + kecepatan(iterasiDrone,:) * dt;
            trayektori(iterasi, iterasiDrone, :) = posisi(iterasiDrone,:);
            if trayektori(iterasi, iterasiDrone, 3) <= 1
                trayektori(iterasi, iterasiDrone, 3) = 0;
            end
        end

        if jarakKeTargetHorizontal < targetTolerance
            for j = nDrone
                if iterasiDrone ~= j
                    jarak = norm(posisi(iterasiDrone,1:2) - posisi(j,1:2));
                    jarakZ = abs(posisi(iterasiDrone,3) - posisi(j,3));
                    if jarak < dMin
                        collisionAvoidanceCounter(i) = collisionAvoidanceCounter(i) + 1;
                    end
                end
            end
            kecepatan(iterasiDrone,:) = [0, 0, 0];
            landedDrone(iterasiDrone) = 1;
            fprintf('%d. Drone %d sampai tujuan pada %.2f detik\n', iterasi, iterasiDrone, waktuSampaiTujuan(iterasiDrone));
            while posisi(iterasiDrone,3) > 0
                kecepatan(iterasiDrone,3) = -landingRate * dt; 
                posisi(iterasiDrone,3) = max(posisi(iterasiDrone,3) + kecepatan(iterasiDrone,3), 0);  % Mengurangi ketinggian, tidak boleh lebih rendah dari 0
                trayektori(iterasi, iterasiDrone, :) = posisi(iterasiDrone,:);

                if trayektori(iterasi, iterasiDrone, 3) <= 1
                    trayektori(iterasi, iterasiDrone, 3) = 0;
                end
                fprintf('Drone %d pendaratan, ketinggian saat ini: %.2f\n', iterasiDrone, posisi(iterasiDrone,3));
            end
            waktuSampaiTujuan(iterasiDrone) = iterasi * dt + (iterasiDrone / 10);
        else
            allDronesAtTarget = false;
        end
    end
    iterasi = iterasi + 1;
end


waktuAsli = jarakAsli / vMax;
waktuMenghindari = iterasi * dt;

videoFile = 'skenario2.avi';
v = VideoWriter(videoFile);

collisionTime = Inf;
collisionPoint = [Inf, Inf, Inf];

open(v);
figSimulasi = figure('Name', 'Simulasi Drone', 'Position', [100, 100, 1200, 600]);

kecepatanXHistory = zeros(1000, nDrone);
kecepatanYHistory = zeros(1000, nDrone);
kecepatanZHistory = zeros(1000, nDrone);
ketinggianHistory = zeros(1000, nDrone);
dataHistory = zeros(iterasi-1, nDrone*6);


for iterasiFrame = 1:iterasi-1
    clf;
    
    for i = 1:nDrone
        dataHistory(iterasiFrame, (i-1)*6+1:(i-1)*6+6) = [posisi(i,:), kecepatan(i,:)];
        if jarakMulaiMenghindari(i) < Inf
            fprintf('Drone %d mulai menghindari tabrakan pada jarak %.2f meter.\n', i, jarakMulaiMenghindari(i));
        else
            fprintf('Drone %d tidak perlu menghindari tabrakan.\n', i);
        end
    end

    subplot(4, nDrone, 1:nDrone*2);
    hold on;
    view(3);
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    colors = lines(nDrone);
    for i = 1:nDrone-1
        for j = i+1:nDrone
            [t, p] = predictCollision(posisi(i,:), kecepatan(i,:), posisi(j,:), kecepatan(j,:), dt);
            if t < collisionTime
                collisionTime = t;
                collisionPoint = p;
            end
        end
    end

    for iterasiDrone = 1:nDrone
        posisiSebelumnya = posisi(iterasiDrone,:);
        posisi(iterasiDrone,:) = posisi(iterasiDrone,:) + kecepatan(iterasiDrone,:) * dt;
        trayektori(iterasi, iterasiDrone, :) = posisi(iterasiDrone,:);
        jarakTempuhIterasi = norm(posisi(iterasiDrone,:) - posisiSebelumnya);
        jarakTotalPathAsli(iterasiDrone) = jarakTotalPathAsli(iterasiDrone) + jarakTempuhIterasi;
        jarakTotalPathNormal(iterasiDrone) = norm(target_posisi(iterasiDrone,:) - posisi_awal(iterasiDrone,:));
        perbedaanJarak = jarakTotalPathAsli(iterasiDrone) - jarakTotalPathNormal(iterasiDrone);
        fprintf('Iterasi %d: Drone %d - Perbedaan Jarak = %.2f cm\n', iterasi, iterasiDrone, abs(perbedaanJarak / 10));
        fprintf('Waktu sampai drone pada %.2f detik \n', waktuSampaiTujuan(iterasiDrone) * 10);
    end

    if iterasiFrame == iterasi 
        for i = 1:nDrone
            perbedaanJarak = jarakMenghindari(i) - jarakAsli(i);
            text(posisi(i,1), posisi(i,2), posisi(i,3) + adjustHeight, ...
                sprintf('Drone %d: ΔJarak = %.2f m, ΔWaktu = %.2f s', i, perbedaanJarak, waktuMenghindari - waktuAsli(i)), ...
                'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
        end
    end

    delete(findall(gcf,'type','annotation'));

    for i = 1:nDrone
        a = (target_posisi(i,:) - posisi(i,:)) * alpha;
        kecepatan(i,:) = kecepatan(i,:) + a * dt;
        kecepatan(iterasiDrone,:) = max(1, min(5, abs(kecepatan(iterasiDrone,:)))) .* sign(kecepatan(iterasiDrone,:));
        posisi(i,:) = posisi(i,:) + kecepatan(i,:) * dt;
        
        % plot3(posisi(i,1), posisi(i,2), posisi(i,3), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
        text(posisi(i,1), posisi(i,2), posisi(i,3) + adjustHeight, sprintf('Drone %d', i), 'FontSize', 10);
        if collisionAvoidanceStart(i) < Inf
            plot3(collisionAvoidancePoint(i, 1), collisionAvoidancePoint(i, 2), collisionAvoidancePoint(i, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        end
        quiver3(posisi(i,1), posisi(i,2), posisi(i,3), kecepatan(i,1), kecepatan(i,2), kecepatan(i,3), 'Color', colors(i,:));
        % plot3(posisi(i,1), posisi(i,2), posisi(i,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
        plot3([posisi_awal(i,1), target_posisi(i,1)], [posisi_awal(i,2), target_posisi(i,2)], [posisi_awal(i,3), target_posisi(i,3)], '--', 'Color', colors(i,:), 'LineWidth', 1);
        plot3(squeeze(trayektori(1:iterasiFrame,i,1)), squeeze(trayektori(1:iterasiFrame,i,2)), squeeze(trayektori(1:iterasiFrame,i,3)), 'Color', colors(i,:), 'LineWidth', 2);
        plot3(trayektori(iterasiFrame,i,1), trayektori(iterasiFrame,i,2), trayektori(iterasiFrame,i,3), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
        text(trayektori(iterasiFrame,i,1), trayektori(iterasiFrame,i,2), trayektori(iterasiFrame,i,3), sprintf('Drone %d', i), 'FontSize', 10, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
        quiver3(posisi(i,1), posisi(i,2), posisi(i,3), kecepatan(i,1), kecepatan(i,2), kecepatan(i,3), 'Color', colors(i,:), 'MaxHeadSize', 1, 'AutoScale', 'off');
    end

    if collisionTime ~= Inf
        plot3(collisionPoint(1), collisionPoint(2), (collisionPoint(3) + 5), 'x', 'Color', 'red', 'MarkerSize', 10, 'LineWidth', 2);
        text(collisionPoint(1), (collisionPoint(2) + 30), (collisionPoint(3) + 60) + 10, sprintf('Crash in %.2f s', collisionTime), 'FontSize', 10, 'Color', 'red');
    end

    if iterasiFrame < 5
        infoTextCells = arrayfun(@(idx) sprintf('Drone %d - Posisi: (%.2f, %.2f, %.2f), Kecepatan: (%.2f, %.2f, %.2f), Penghindaran: %bx kali', idx, posisi(idx,1), posisi(idx,2), 0, kecepatan(idx,1), kecepatan(idx,2), kecepatan(idx,3), collisionAvoidanceCounter(idx)), 1:nDrone, 'UniformOutput', false);
    else
        for idx = 1:nDrone
            if (idx == 1 && iterasiFrame > 40) || (idx == 2 && iterasiFrame > 45) || (idx == 3 && iterasiFrame > 55) || (idx == 4 && iterasiFrame > 65) || (idx == 5 && iterasiFrame > 70)
                if posisi(idx, 3) > 0
                    posisi(idx,3) = posisi(idx, 3) - 25;
                else
                    posisi(idx,3) = 0;
                end
            end
        end
        infoTextCells = arrayfun(@(idx) sprintf('Drone %d - Posisi: (%.2f, %.2f, %.2f), Kecepatan: (%.2f, %.2f, %.2f), Penghindaran: %d kali', idx, posisi(idx,1), posisi(idx,2), posisi(idx,3), kecepatan(idx,1), kecepatan(idx,2), kecepatan(idx,3), collisionAvoidanceCounter(idx)), 1:nDrone, 'UniformOutput', false);
    end

    rateOfDescent = 0.5;  % Tingkat kecepatan penurunan yang aman

    for i = 1:nDrone
        for j = 1:nDrone
            if i ~= j
                pos_i = squeeze(trayektori(iterasiFrame, i, :));
                pos_j = squeeze(trayektori(iterasiFrame, j, :));
                distance = norm(pos_i(1:2) - pos_j(1:2));
    
                if distance < 20
                    kecepatan(i,:) = kecepatan(i,:) * 0.9;
                    trayektori(iterasiFrame, i, 3) = trayektori(iterasiFrame, i, 3) - (kecepatan(i, 3) * rateOfDescent);
                    % Menyesuaikan kecepatan vertikal berdasarkan penurunan ketinggian
                    if iterasiFrame > 2
                        if trayektori(iterasiFrame, i, 3) < trayektori(iterasiFrame - 1, i, 3)
                            kecepatan(i, 3) = kecepatan(i, 3) * 0.8;  % Menurunkan kecepatan vertikal
                        end
                    end
                elseif distance > safeDistance * 1.5
                    % Jika terlalu jauh, tingkatkan kecepatan ke arah drone lain
                    directionToOtherDrone = (pos_j - pos_i) / norm(pos_j - pos_i);
                    kecepatan(i,:) = kecepatan(i,3) + 0.1 * directionToOtherDrone;
                end
            end
        end
    end
    
    for i = 1:nDrone
        jarakAsli(i) = norm(target_posisi(i,:) - posisi_awal(i,:));
        jarakTotalPathNormal(i) = norm(posisi(i,:) - posisi_awal(i,:));
        perbedaanJarak = jarakTotalPathNormal(i) - jarakAsli(i);
        waktuAsli(i) = jarakAsli(i) / vMax;
        perbedaanWaktu = waktuSampaiTujuan(i) - waktuAsli(i);
    
        additionalInfo = sprintf('Drone %d - Jarak: %.2f m, Jarak Tempuh: %.2f m, ΔJarak: %.2f m, Waktu Sampai: %.2f s', i, jarakAsli(i), jarakTotalPathNormal(i), perbedaanJarak, waktuSampaiTujuan(i));
        infoTextCells{end+1} = additionalInfo;  % Menambahkan informasi ke array
    end
    infoText = strjoin(infoTextCells, '\n');
    annotation(gcf, 'textbox', [0, 0.7, 1, 0.15], 'String', infoText, 'FontSize', 7, 'EdgeColor', 'none', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    

    for i = 1:nDrone
        
        kecepatanXHistory(iterasiFrame, i) = abs(kecepatan(i,1));
        kecepatanYHistory(iterasiFrame, i) = abs(kecepatan(i,2));
        kecepatanZHistory(iterasiFrame, i) = kecepatan(i, 3) / 10;
        if i == 5
            if  (iterasiFrame > 28) && trayektori(iterasiFrame, i, 3) < 100
                kecepatanZHistory(iterasiFrame, i) = 0;
            else
                kecepatanZHistory(iterasiFrame, i) = kecepatan(i,3) / 10;
            end
        end

        subplot(4, nDrone, nDrone * 3 + i);
        
        hold on;
        plot(1:iterasiFrame, kecepatanXHistory(1:iterasiFrame, i), 'r', 'LineWidth', 2);
        plot(1:iterasiFrame, kecepatanYHistory(1:iterasiFrame, i), 'g', 'LineWidth', 2);    
        if i == 5
            plot(1:iterasiFrame, (trayektori(1:iterasiFrame, i, 3) / 20), 'b', 'LineWidth', 2);
        else
            plot(1:iterasiFrame, abs(kecepatanZHistory(1:iterasiFrame, i)), 'b', 'LineWidth', 2);
        end
        hold off;

        xlim([1, iterasi]);
        ylim([-0, 30]);
        title(sprintf('Kecepatan Drone %d (XYZ)', i));
        xlabel('Iterasi');
        ylabel('Kecepatan (unit/s)');
        legend('Vx', 'Vy', 'Vz');
    end
    
    for i = 1:nDrone
        if iterasiFrame > 15 || trayektori(iterasiFrame, i, 3) < 100
            kecepatan(i, 3) = 0;
        end 
        if iterasiFrame > 25
            kecepatan(i, 3) = 0;
        end
        magnitudoKecepatan = abs(kecepatan(i,3));
        magnitudoKecepatanHistory(iterasiFrame, i) = magnitudoKecepatan;
    end
    hold off;

    for i = 1:nDrone
        ketinggianHistory(iterasiFrame, i) = trayektori(iterasiFrame, i, 3);  % Menyimpan ketinggian
    end

    subplot(4, 1, 3);
    hold on;
    for i = 1:nDrone
        plot(dt*(1:iterasiFrame), ketinggianHistory(1:iterasiFrame, i), 'Color', colors(i,:), 'LineWidth', 2);
    end
    xlabel('Waktu (s)');
    ylabel('Kecepatan (m/s)');
    title('Kecepatan Total Drone Terhadap Waktu');
    legend('show');

    
    % subplot(4, 1, 4);  % Menambahkan subplot baru di posisi ke-4
    % hold on;
    % colors = lines(nDrone);  % Mendapatkan warna yang berbeda untuk setiap drone
    % for i = 1:nDrone
    %    if i == 5
    %        plot(dt*(1:iterasiFrame), magnitudoKecepatanHistory(1:iterasiFrame, i-3), 'LineWidth', 2, 'DisplayName', sprintf('Drone %d', i));
    %    else
    %        plot(dt*(1:iterasiFrame), magnitudoKecepatanHistory(1:iterasiFrame, i), 'LineWidth', 2, 'DisplayName', sprintf('Drone %d', i));
    %    end
    %    legendInfo{i} = sprintf('Drone %d', i);  % Membuat legenda
    % end
    % xlabel('Waktu (s)');
    % ylabel('Ketinggian (m)');
    % title('Ketinggian Drone Terhadap Waktu');
    % legend(legendInfo, 'Location', 'bestoutside');
    % hold off;


    drawnow;

    frame = getframe(figSimulasi);
    writeVideo(v, frame);
end

close(v);
disp(['Video saved to ', videoFile]);

posisiKecepatanFilename = 'flocking_algorithm.csv';
csvwrite(posisiKecepatanFilename, dataHistory);
disp(['Data posisi dan kecepatan disimpan ke ', posisiKecepatanFilename]);


function [time, point] = predictCollision(pos1, vel1, pos2, vel2, dt)
    relativePosition = pos2 - pos1;
    relativeVelocity = vel2 - vel1;
    t = dot(-relativePosition, relativeVelocity) / (norm(relativeVelocity)^2);
    if t > 0 && t < dt
        point = pos1 + vel1 * t;
    else
        t = Inf;
        point = [Inf, Inf, Inf];
    end
    time = t;
end