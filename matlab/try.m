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
takeoffRate = 0.5;
landingRate = 0.5;
decelerationRate = 0.5;

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
        elseif jarakKeTargetHorizontal <= targetTolerance
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


figure;
hold on;
grid on;
axis equal;

% Colors for each drone
colors = lines(nDrone);

% Plot initial positions
for i = 1:nDrone
    plot3(posisi_awal(i,1), posisi_awal(i,2), posisi_awal(i,3), 'o', 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
end

% Plot target positions
for i = 1:nDrone
    plot3(target_posisi(i,1), target_posisi(i,2), target_posisi(i,3), 'x', 'MarkerEdgeColor', colors(i,:), 'MarkerSize', 10);
end

% Plot trajectories for each drone
for i = 1:nDrone
    plot3(trayektori(:,i,1), trayektori(:,i,2), trayektori(:,i,3), 'Color', colors(i,:), 'LineWidth', 2);
end

% Label axes
xlabel('X Position');
ylabel('Y Position');
zlabel('Altitude');
view(3);
% Title and Legend
title('3D Trajectories of Drones');
legend(arrayfun(@(x) sprintf('Drone %d', x), 1:nDrone, 'UniformOutput', false));

hold off;
