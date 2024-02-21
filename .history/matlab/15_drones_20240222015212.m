% Jumlah drone
nDrone = 15;

% Membuat posisi awal dan target posisi secara dinamis
% Menggunakan ruang 400x200 dengan ketinggian 150
posisi_awal = zeros(nDrone, 3);
target_posisi = zeros(nDrone, 3);

% Mengatur posisi awal dan target secara manual untuk demo
for i = 1:nDrone
    if mod(i, 3) == 0
        posisi_awal(i, :) = [0, (i-1)*13.33, 150];
        target_posisi(i, :) = [400, 200 - (i-1)*13.33, 150];
    elseif mod(i, 3) == 1
        posisi_awal(i, :) = [400, (i-1)*13.33, 150];
        target_posisi(i, :) = [0, 200 - (i-1)*13.33, 150];
    else
        posisi_awal(i, :) = [200, (i-1)*13.33, 150];
        target_posisi(i, :) = [200, 200 - (i-1)*13.33, 150];
    end
end
             
% Jumlah iterasi simulasi
n_iterasi = 100;

% Parameter flocking
dMin = 50; % Jarak minimal untuk penghindaran tabrakan
vMax = 20; % Kecepatan maksimum drone

% Inisialisasi posisi dan kecepatan
posisi = posisi_awal;
kecepatan = zeros(nDrone,3);

% Untuk menyimpan trayektori
trayektori = zeros(n_iterasi, nDrone, 3);

for t = 1:n_iterasi
    for i = 1:nDrone
        % Hitung vektor menuju target
        vTarget = (target_posisi(i,:) - posisi(i,:));
        vTarget = vTarget / norm(vTarget) * vMax;
        
        % Penghindaran tabrakan
        for j = 1:nDrone
            if i ~= j
                jarak = norm(posisi(i,:) - posisi(j,:));
                if jarak < dMin
                    vAvoid = posisi(i,:) - posisi(j,:);
                    vTarget = vTarget + vAvoid;
                end
            end
        end
        
        % Batasi kecepatan
        if norm(vTarget) > vMax
            vTarget = vTarget / norm(vTarget) * vMax;
        end
        
        % Update posisi dan kecepatan
        kecepatan(i,:) = vTarget;
        posisi(i,:) = posisi(i,:) + kecepatan(i,:);
    end
    
    % Simpan trayektori
    trayektori(t,:,:) = posisi;
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
title('3D Trayektori Drone dengan Flocking Algorithm');
