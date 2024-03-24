function collision()
    nDrone = 5;
    dMin = 50;
    vMax = 20;
    adjustHeight = 20;
    gravitasi = 9.81;
    massa = 1.0;
    k = 0.1;

    alpha = k * (gravitasi / massa);
    targetTolerance = 10.0;

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

    posisi = posisi_awal;
    kecepatan = zeros(nDrone,3);

    dt = 0.05;
    iterasi = 1;
    allDronesAtTarget = false;

    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    xlim([min(posisi_awal(:,1))-100, max(target_posisi(:,1))+100]);
    ylim([min(posisi_awal(:,2))-100, max(target_posisi(:,2))+100]);
    zlim([0, max(posisi_awal(:,3))+200]);

    while ~allDronesAtTarget
        clf;
        hold on;
        grid on;
        axis equal;
        xlabel('X');
        ylabel('Y');
        view(3);
        zlabel('Z');
        xlim([min(posisi_awal(:,1))-100, max(target_posisi(:,1))+100]);
        ylim([min(posisi_awal(:,2))-100, max(target_posisi(:,2))+100]);
        zlim([0, max(posisi_awal(:,3))+200]);

        allDronesAtTarget = true;
        for i = 1:nDrone
            vTarget = (target_posisi(i,:) - posisi(i,:)) * alpha;

            % Cek dan hindari tabrakan
            for j = 1:nDrone
                if j ~= i
                    if predictCollision(posisi(i,:), kecepatan(i,:), posisi(j,:), kecepatan(j,:), dt)
                        vTarget = avoidCollision(posisi(i,:), target_posisi(i,:), posisi(j,:));
                        break;
                    end
                end
            end

            kecepatan(i,:) = kecepatan(i,:) + vTarget * dt;
            if norm(kecepatan(i,:)) > vMax
                kecepatan(i,:) = kecepatan(i,:) / norm(kecepatan(i,:)) * vMax;
            end

            posisi(i,:) = posisi(i,:) + kecepatan(i,:) * dt;
            if norm(posisi(i,:) - target_posisi(i,:)) >= targetTolerance
                allDronesAtTarget = false;
            end

            plot3(posisi(i,1), posisi(i,2), posisi(i,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
           
            line([posisi_awal(i,1), target_posisi(i,1)], [posisi_awal(i,2), target_posisi(i,2)], [posisi_awal(i,3), target_posisi(i,3)], 'Color', 'red', 'LineStyle', '--');
        end

        drawnow;

        iterasi = iterasi + 1;
    end

    hold off;
    disp('Simulasi Selesai');
end

function isCollision = predictCollision(pos1, vel1, pos2, vel2, dt)
    % Sederhana prediksi tabrakan
    futurePos1 = pos1 + vel1 * dt;
    futurePos2 = pos2 + vel2 * dt;
    isCollision = norm(futurePos1 - futurePos2) < 50; % dMin is hardcoded here, adjust as necessary
end

function vAvoid = avoidCollision(currentPos, targetPos, obstaclePos)
    % Sederhana mekanisme penghindaran
    avoidDir = currentPos - obstaclePos;
    avoidDir(3) = 0; % Hindari perubahan ketinggian secara drastis
    nVec = avoidDir / norm(avoidDir); % Normalized direction vector for avoidance
    vAvoid = nVec * 20; % vMax is hardcoded here, adjust as necessary
end
