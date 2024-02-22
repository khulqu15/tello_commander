% Jumlah drone
nDrone = 15;

% Parameter flocking dan kondisi awal
dMin = 50; % Jarak minimal untuk penghindaran tabrakan
vMax = 20; % Kecepatan maksimum drone
adjustHeight = 20; % Ketinggian penyesuaian untuk menghindari tabrakan

% Inisialisasi posisi, kecepatan, dan trayektori
posisi = posisi_awal;
kecepatan = zeros(nDrone,3);
iterasi = 0;
maxIterasi = 100;

while iterasi < maxIterasi
    iterasiDrone = 1;
    while iterasiDrone <= nDrone
        % Hitung vektor menuju target
        vTarget = (target_posisi(iterasiDrone,:) - posisi(iterasiDrone,:));
        norm_vTarget = norm(vTarget(1:2)); % Norm horizontal
        if norm_vTarget > 0
            vTarget(1:2) = vTarget(1:2) / norm_vTarget * vMax;
        end
        
        % Inisialisasi penghindaran tabrakan
        vAvoid = zeros(1,3);
        droneToAvoid = 0;
        
        j = 1;
        while j <= nDrone
            if iterasiDrone ~= j
                jarak = norm(posisi(iterasiDrone,1:2) - posisi(j,1:2)); % Jarak horizontal
                jarakZ = abs(posisi(iterasiDrone,3) - posisi(j,3)); % Jarak vertikal
                
                % Jika dalam jarak penghindaran dan vertikal lebih kecil dari batas
                if jarak < dMin && jarakZ < adjustHeight
                    % Tentukan apakah perlu naik atau turun
                    if posisi(iterasiDrone,3) <= posisi(j,3)
                        vAvoid(3) = vAvoid(3) - vMax; % Turun
                    else
                        vAvoid(3) = vAvoid(3) + vMax; % Naik
                    end
                    droneToAvoid = droneToAvoid + 1;
                end
            end
            j = j + 1;
        end
        
        % Rata-rata penghindaran jika ada lebih dari satu drone untuk dihindari
        if droneToAvoid > 0
            vAvoid(3) = vAvoid(3) / droneToAvoid;
        end
        
        % Gabungkan vektor target dengan penghindaran
        vTarget = vTarget + vAvoid;
        
        % Batasi kecepatan total
        if norm(vTarget) > vMax
            vTarget = vTarget / norm(vTarget) * vMax;
        end
        
        % Update posisi dan kecepatan
        kecepatan(iterasiDrone,:) = vTarget;
        posisi(iterasiDrone,:) = posisi(iterasiDrone,:) + kecepatan(iterasiDrone,:);
        
        iterasiDrone = iterasiDrone + 1;
    end
    
    % Update iterasi dan kondisi simulasi
    iterasi = iterasi + 1;
end

% Visualisasi
figure;
hold on;
grid on;
axis equal;

colors = lines(nDrone); % Menghasilkan warna berbeda untuk setiap drone

for i = 1:nDrone
    % Trayektori drone
    plot3(squeeze(trayektori(:,i,1)), squeeze(trayektori(:,i,2)), squeeze(trayektori(:,i,3)), 'Color', colors(i,:), 'LineWidth', 2);
end

% Tandai posisi awal dan akhir
plot3(posisi_awal(:,1), posisi_awal(:,2), posisi_awal(:,3), 'ko', 'MarkerFaceColor', 'k');
plot3(target_posisi(:,1), target_posisi(:,2), target_posisi(:,3), 'go', 'MarkerFaceColor', 'g');

view(3); % Atur sudut pandang 3D
legend(arrayfun(@(x) sprintf('Drone %d', x), 1:nDrone, 'UniformOutput', false), 'Location', 'Best');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trayektori Drone dengan Flocking Algorithm untuk Penghindaran Tabrakan');
