% Jumlah drone awal
nDrone = 5;

% position awal drone, jika kurang tambahkan dimensi X, Y, Z
start_position = [0, 0, 150;
               400, 0, 150;
               0, 200, 150;
               400, 200, 150;
               200, 100, 150];

% Target position yang membuat drone memutar ruangan dan berpotensi bertemu
% Usahakan dimensi array sesuai dengan start_position
target_position = [400, 200, 150;
                 0, 200, 150;
                 400, 0, 150;
                 0, 0, 150;
                 200, 0, 150];
             
% Jumlah iterasi simulasi
n_iteration = 100;

% Parameter flocking
dMin = 50; % Jarak minimal untuk penghindaran tabrakan
vMax = 20; % velocity maksimum drone

% Inisialisasi position dan velocity
position = start_position;
velocity = zeros(nDrone,3);

% Untuk menyimpan trajectories
trajectories = zeros(n_iteration, nDrone, 3);

for t = 1:n_iteration
    for i = 1:nDrone
        % Hitung vektor menuju target
        vTarget = (target_position(i,:) - position(i,:));
        vTarget = vTarget / norm(vTarget) * vMax;
        
        % Penghindaran tabrakan
        for j = 1:nDrone
            if i ~= j
                jarak = norm(position(i,:) - position(j,:));
                if jarak < dMin
                    vAvoid = position(i,:) - position(j,:);
                    vTarget = vTarget + vAvoid;
                end
            end
        end
        
        % Batasi velocity
        if norm(vTarget) > vMax
            vTarget = vTarget / norm(vTarget) * vMax;
        end
        
        % Update position dan velocity
        velocity(i,:) = vTarget;
        position(i,:) = position(i,:) + velocity(i,:);
    end
    
    % Simpan trajectories
    trajectories(t,:,:) = position;
end

% Visualisasi
figure;
hold on;
grid on;
axis equal;

colors = lines(nDrone); % Menghasilkan warna berbeda untuk setiap drone

for i = 1:nDrone
    % trajectories drone
    plot3(squeeze(trajectories(:,i,1)), squeeze(trajectories(:,i,2)), squeeze(trajectories(:,i,3)), 'Color', colors(i,:), 'LineWidth', 2);
end

% Tandai position awal dan akhir
plot3(start_position(:,1), start_position(:,2), start_position(:,3), 'ko', 'MarkerFaceColor', 'k');
plot3(target_position(:,1), target_position(:,2), target_position(:,3), 'go', 'MarkerFaceColor', 'g');

view(3); % Atur sudut pandang 3D
legend(arrayfun(@(x) sprintf('Drone %d', x), 1:nDrone, 'UniformOutput', false), 'Location', 'Best');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D trajectories Drone Flocking Algorithm');