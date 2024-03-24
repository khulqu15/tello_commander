function collision()
    nDrone = 5;
    dMin = 50;
    vMax = 20;
    gravitasi = 9.81;
    massa = 1.0;
    k = 0.1;

    alpha = k * (gravitasi / massa);
    targetTolerance = 10.0;
    kecepatan = zeros(nDrone, 3);
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
        [vTarget, isAtTarget] = updateDronePosition(posisi(i,:), target_posisi(i,:), vMax, dt);
        kecepatan(i,:) = vTarget;
        posisi(i,:) = posisi(i,:) + vTarget * dt;
        for j = 1:nDrone

            [collisionTime, collisionPoint] = predictCollisionTimeAndPoint(posisi(i,:), kecepatan(i,:), posisi(j,:), kecepatan(j,:), dMin);
            if ~isinf(collisionTime)
                plot3(collisionPoint(1), collisionPoint(2), collisionPoint(3), 'x', 'MarkerSize', 10, 'Color', 'red');
                text(collisionPoint(1), collisionPoint(2), collisionPoint(3)+100, sprintf('T-%0.2fs', collisionTime), 'Color', 'red');
            end
            
            if j ~= i
                [isCollision, timeToCollision, collisionPoint] = predictCollision(posisi(i,:), kecepatan(i,:), posisi(j,:), kecepatan(j,:), dt, dMin);
                if isCollision && timeToCollision <= dt
                    vTarget = avoidCollision(posisi(i,:), target_posisi(i,:), posisi(j,:));
                    plot3(collisionPoint(1), collisionPoint(2), collisionPoint(3), 'x', 'MarkerSize', 10, 'MarkerEdgeColor', 'r');
                    text(collisionPoint(1), collisionPoint(2), collisionPoint(3)+5, sprintf('%.2f s', timeToCollision), 'Color', 'red');
                    break;
                end
            end
        end
        plotDronePath(posisi_awal(i,:), target_posisi(i,:)); % Fungsi visualisasi lintasan drone

        kecepatan(i,:) = kecepatan(i,:) + vTarget * dt;
        if norm(kecepatan(i,:)) > vMax
            kecepatan(i,:) = kecepatan(i,:) / norm(kecepatan(i,:)) * vMax;
        end

        posisi(i,:) = posisi(i,:) + kecepatan(i,:) * dt;
        if norm(posisi(i,:) - target_posisi(i,:)) >= targetTolerance
            allDronesAtTarget = false;
        end

        plot3(posisi(i,1), posisi(i,2), posisi(i,3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        % line([posisi_awal(i,1), target_posisi(i,1)], [posisi_awal(i,2), target_posisi(i,2)], [posisi_awal(i,3), target_posisi(i,3)], 'Color', 'red', 'LineStyle', '--');
    end

    drawnow;

    iterasi = iterasi + 1;
end

hold off;
disp('Simulasi Selesai');

end

function [collisionTime, collisionPoint] = predictCollisionTimeAndPoint(pos1, vel1, pos2, vel2, dMin)
    dp = pos2 - pos1;
    dv = vel2 - vel1;
    dv2 = dot(dv, dv);
    dpdv = dot(dp, dv);
    discriminant = dpdv^2 - dv2 * (dot(dp, dp) - dMin^2);

    if discriminant < 0
        collisionTime = Inf;
        collisionPoint = [NaN, NaN, NaN];
    else
        t1 = (-dpdv + sqrt(discriminant)) / dv2;
        t2 = (-dpdv - sqrt(discriminant)) / dv2;
        collisionTime = min(t1, t2);
        if collisionTime < 0
            collisionTime = Inf;
            collisionPoint = [NaN, NaN, NaN];
        else
            collisionPoint = pos1 + vel1 * collisionTime;
        end
    end
end


function [isCollision, timeToCollision, collisionPoint] = predictCollision(pos1, vel1, pos2, vel2, dt, dMin)
    relPos = pos2 - pos1;
    relVel = vel2 - vel1;
    relSpeedSq = dot(relVel, relVel);
    approachRate = dot(relPos, relVel);
    
    isCollision = false;
    timeToCollision = Inf;
    collisionPoint = [Inf, Inf, Inf];
    
    if approachRate < 0
        closestApproach = dot(relPos, relPos) - (approachRate^2) / relSpeedSq;
        if closestApproach < (dMin^2)
            isCollision = true;
            timeToCollision = -approachRate / relSpeedSq;
            collisionPoint = pos1 + vel1 * timeToCollision;
        end
    end
end

function [vTarget, isAtTarget] = updateDronePosition(posisi, target_posisi, vMax, dt)
    direction = target_posisi - posisi;
    distance = norm(direction);
    isAtTarget = distance < 10;
    if isAtTarget
        vTarget = [0, 0, 0];
    else
        vTarget = (direction / distance) * vMax;
    end
end


function plotDronePath(posisi_awal, target_posisi)
    plot3([posisi_awal(1), target_posisi(1)], [posisi_awal(2), target_posisi(2)], [posisi_awal(3), target_posisi(3)], 'Color', 'blue', 'LineStyle', '--');
    plot3([posisi_awal(1), target_posisi(1)], [posisi_awal(2), target_posisi(2)], [posisi_awal(3), target_posisi(3)], '--', 'Color', [0.5, 0.5, 0.5]);
end

function vAvoid = avoidCollision(currentPos, targetPos, obstaclePos)
    avoidDir = currentPos - obstaclePos;
    avoidDir(3) = 0;
    nVec = avoidDir / norm(avoidDir);
    vAvoid = nVec * 20;
end
