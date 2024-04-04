nDrone = 5;
dMin = 50;
vMax = 20;
adjustHeight = 20;
gravitasi = 9.81;
massa = 1.0;
k = 0.1;

alpha = k * (gravitasi / massa);
targetTolerance = 1.0;

jarakAsli = zeros(1, nDrone);
jarakMenghindari = zeros(1, nDrone);

magnitudoKecepatanHistory = zeros(1000, nDrone);
waktuSampaiTujuan = zeros(1, nDrone);

jarakMulaiMenghindari = Inf * ones(1, nDrone);

posisi_awal = zeros(nDrone, 3);
target_posisi = zeros(nDrone, 3);
for i = 1:nDrone
    jarakAsli(i) = norm(target_posisi(i,:) - posisi_awal(i,:));
    if mod(i, 3) == 0
        posisi_awal(i, :) = [0, (i-1)*13.33, 0];
        target_posisi(i, :) = [200, 200, 150 + (i-1)*5];
    elseif mod(i, 3) == 1
        posisi_awal(i, :) = [400, (i-1)*13.33, 0];
        target_posisi(i, :) = [200, 200, 150 + (i-1)*5];
    else
        posisi_awal(i, :) = [200, (i-1)*13.33, 0];
        target_posisi(i, :) = [200, 200, 150 + (i-1)*5];
    end
end

posisi = posisi_awal;
kecepatan = zeros(nDrone,3);
trayektori = zeros(1000, nDrone, 3);
iterasi = 1;
allDronesAtTarget = false;

dt = 0.05;
distanceTrajectory = zeros(1, nDrone);
ketinggianTakeoff = 50; % Drone mulai meningkatkan ketinggian hingga 50 meter
ketinggianTarget = 150; % Ketinggian target untuk penerbanga
collisionAvoidanceStart = Inf * ones(1, nDrone);
collisionAvoidancePoint = Inf * ones(nDrone, 3);
jarakTotalPathAsli = zeros(1, nDrone);
jarakTotalPathNormal = zeros(1, nDrone);

while ~allDronesAtTarget
    allDronesAtTarget = true;
    for iterasiDrone = 1:nDrone
        jarakKeTargetHorizontal = norm(target_posisi(iterasiDrone,1:2) - posisi(iterasiDrone,1:2));
        jarakKeTargetVertical = abs(target_posisi(iterasiDrone,3) - posisi(iterasiDrone,3));

        % Takeoff: Naikkan ketinggian drone jika masih di bawah ketinggian takeoff
        if posisi(iterasiDrone,3) < ketinggianTakeoff
            kecepatan(iterasiDrone,3) = takeoffRate * dt;
        elseif posisi(iterasiDrone,3) >= ketinggianTakeoff && posisi(iterasiDrone,3) < ketinggianTarget
            posisi(iterasiDrone,3) = ketinggianTarget; % Set ketinggian ke ketinggian target setelah takeoff
        end

        if jarakKeTargetHorizontal > targetTolerance
            % Penghitungan kecepatan horizontal berdasarkan target
            vTargetHorizontal = (target_posisi(iterasiDrone,1:2) - posisi(iterasiDrone,1:2)) * alpha;
            kecepatan(iterasiDrone,1:2) = kecepatan(iterasiDrone,1:2) + vTargetHorizontal * dt;
            if norm(kecepatan(iterasiDrone,1:2)) > vMax
                kecepatan(iterasiDrone,1:2) = (kecepatan(iterasiDrone,1:2) / norm(kecepatan(iterasiDrone,1:2))) * vMax;
            end
        else
            % Landing: Turunkan ketinggian jika sudah dekat dengan target dan pada ketinggian target
            if posisi(iterasiDrone,3) > posisi_awal(iterasiDrone,3) && jarakKeTargetVertical < ketinggianTarget
                kecepatan(iterasiDrone,3) = -landingRate * dt; % Turunkan ketinggian
            end
        end
        
        vTarget = target_posisi(iterasiDrone,:) - posisi(iterasiDrone,:);
        a = (vTarget - kecepatan(iterasiDrone,:)) * alpha;
        
        kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) + a * dt;
        if norm(kecepatan(iterasiDrone,:)) > vMax
            kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) / norm(kecepatan(iterasiDrone,:)) * vMax;
        elseif jarakKeTarget <= targetTolerance
            kecepatan(iterasiDrone,:) = max(kecepatan(iterasiDrone,:) - decelerationRate * dt, 0);
        end

        if norm(kecepatan(iterasiDrone,:)) < vMax && jarakKeTarget > targetTolerance
            kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) + ((target_posisi(iterasiDrone,:) - posisi(iterasiDrone,:)) / norm(target_posisi(iterasiDrone,:) - posisi(iterasiDrone,:))) * alpha * dt;
        end

        if jarakKeTarget <= targetTolerance && norm(kecepatan(iterasiDrone,:)) > 0
            kecepatanReductionFactor = max(0, 1 - decelerationRate * dt); % Faktor pengurangan kecepatan
            kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) * kecepatanReductionFactor;
        end
        
        posisi(iterasiDrone,:) = posisi(iterasiDrone,:) + kecepatan(iterasiDrone,:) * dt;

        vAvoid = zeros(1,3);
        for j = 1:nDrone
            if iterasiDrone ~= j
                jarak = norm(posisi(iterasiDrone,1:2) - posisi(j,1:2));
                jarakZ = abs(posisi(iterasiDrone,3) - posisi(j,3));
                if jarak < dMin && jarakZ < adjustHeight
                    if iterasi < collisionAvoidanceStart(iterasiDrone)
                        collisionAvoidanceStart(iterasiDrone) = iterasi;
                        collisionAvoidancePoint(iterasiDrone, :) = posisi(iterasiDrone, :);
                        jarakMulaiMenghindari(iterasiDrone) = jarak;
                    end
                    direction = (posisi(j,:) - posisi(iterasiDrone,:)) / norm(posisi(j,:) - posisi(iterasiDrone,:));
                    inertiaFactor = massa / (1 + alpha);
                    vAvoid = vAvoid + direction * vMax * inertiaFactor;
                end
            end
        end
        
        vNew = vTarget + vAvoid;
        kecepatan(iterasiDrone,:) = kecepatan(iterasiDrone,:) + vNew * alpha;
        
        posisi(iterasiDrone,:) = posisi(iterasiDrone,:) + kecepatan(iterasiDrone,:) * dt;
        trayektori(iterasi, iterasiDrone, :) = posisi(iterasiDrone,:);
        
        if jarakKeTargetHorizontal < targetTolerance
            kecepatan(iterasiDrone,:) = [0, 0, 0];
            if allDronesAtTarget
                waktuSampaiTujuan(iterasiDrone) = iterasi * dt;
                fprintf('%d. Drone %d sampai tujuan pada %.2f detik\n', iterasi, iterasiDrone, waktuSampaiTujuan(iterasiDrone));
            end
        else
            allDronesAtTarget = false;
        end
    end
    if allDronesAtTarget
        fprintf('Semua drone telah mencapai target pada iterasi ke-%d.\n', iterasi);
        break;
    end
    iterasi = iterasi + 1;
end


waktuAsli = jarakAsli / vMax;
waktuMenghindari = iterasi * dt;

videoFile = 'drone_simulation_5_with_paths.avi';
v = VideoWriter(videoFile);

collisionTime = Inf;
collisionPoint = [Inf, Inf, Inf];

open(v);
figSimulasi = figure('Name', 'Simulasi Drone', 'Position', [100, 100, 1200, 600]);

kecepatanXHistory = zeros(1000, nDrone);
kecepatanYHistory = zeros(1000, nDrone);
kecepatanZHistory = zeros(1000, nDrone);

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
        % Perbarui posisi drone berdasarkan kecepatan saat ini
        posisiSebelumnya = posisi(iterasiDrone,:);
        posisi(iterasiDrone,:) = posisi(iterasiDrone,:) + kecepatan(iterasiDrone,:) * dt;

        % Hitung dan tambahkan jarak yang ditempuh pada iterasi ini ke total jarak
        jarakTempuhIterasi = norm(posisi(iterasiDrone,:) - posisiSebelumnya);
        jarakTotalPathAsli(iterasiDrone) = jarakTotalPathAsli(iterasiDrone) + jarakTempuhIterasi;

        % Hitung jarak total yang seharusnya ditempuh pada path normal (langsung ke target) dan perbarui
        jarakTotalPathNormal(iterasiDrone) = norm(target_posisi(iterasiDrone,:) - posisi_awal(iterasiDrone,:));

        % Hitung perbedaan jarak antara path asli dengan path normal
        perbedaanJarak = jarakTotalPathAsli(iterasiDrone) - jarakTotalPathNormal(iterasiDrone);

        % Tampilkan informasi perbedaan jarak
        fprintf('Iterasi %d: Drone %d - Perbedaan Jarak = %.2f cm\n', iterasi, iterasiDrone, abs(perbedaanJarak / 10));
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
        
        plot3(posisi(i,1), posisi(i,2), posisi(i,3), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
        text(posisi(i,1), posisi(i,2), posisi(i,3) + adjustHeight, sprintf('Drone %d', i), 'FontSize', 10);
        if collisionAvoidanceStart(i) < Inf
            plot3(collisionAvoidancePoint(i, 1), collisionAvoidancePoint(i, 2), collisionAvoidancePoint(i, 3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        end
        quiver3(posisi(i,1), posisi(i,2), posisi(i,3), kecepatan(i,1), kecepatan(i,2), kecepatan(i,3), 'Color', colors(i,:));
        plot3(posisi(i,1), posisi(i,2), posisi(i,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', colors(i,:));
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

    infoTextCells = arrayfun(@(idx) sprintf('Drone %d - Posisi: (%.2f, %.2f, %.2f), Kecepatan: (%.2f, %.2f, %.2f)', idx, posisi(idx,1), posisi(idx,2), posisi(idx,3), kecepatan(idx,1), kecepatan(idx,2), kecepatan(idx,3)), 1:nDrone, 'UniformOutput', false);
    infoText = strjoin(infoTextCells, '\n');
    annotation(gcf, 'textbox', [0, 0.85, 1, 0.15], 'String', infoText, 'FontSize', 7, 'EdgeColor', 'none', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom');
    

    for i = 1:nDrone
        kecepatanXHistory(iterasiFrame, i) = kecepatan(i,1);
        kecepatanYHistory(iterasiFrame, i) = kecepatan(i,2);
        kecepatanZHistory(iterasiFrame, i) = kecepatan(i,3);

        subplot(4, nDrone, nDrone * 3 + i);
        plot(1:iterasiFrame, kecepatanXHistory(1:iterasiFrame, i), 'r', 'LineWidth', 2);
        hold on;
        plot(1:iterasiFrame, kecepatanYHistory(1:iterasiFrame, i), 'g', 'LineWidth', 2);
        plot(1:iterasiFrame, kecepatanZHistory(1:iterasiFrame, i), 'b', 'LineWidth', 2);
        hold off;

        xlim([1, iterasi]);
        ylim([-vMax, vMax]);
        title(sprintf('Kecepatan Drone %d (XYZ)', i));
        xlabel('Iterasi');
        ylabel('Kecepatan (unit/s)');
        legend('Vx', 'Vy', 'Vz');
    end
    
    for i = 1:nDrone
        magnitudoKecepatan = abs(kecepatan(i,3));
        magnitudoKecepatanHistory(iterasiFrame, i) = magnitudoKecepatan;
    end
    subplot(4, 1, 3);
    hold on;
    for i = 1:nDrone
        plot(dt*(1:iterasiFrame), magnitudoKecepatanHistory(1:iterasiFrame, i), 'LineWidth', 2, 'DisplayName', sprintf('Drone %d', i));
    end
    xlabel('Waktu (s)');
    ylabel('Kecepatan (m/s)');
    title('Kecepatan Total Drone Terhadap Waktu');
    legend('show');

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