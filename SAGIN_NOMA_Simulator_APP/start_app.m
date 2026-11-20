function start_app()
    close all;
    clear classes;
    clc;

    % Tự động thêm toàn bộ thư mục con vào Search Path của MATLAB
    projectRoot = fileparts(mfilename('fullpath'));
    addpath(genpath(projectRoot));
    rehash;

    % Khởi chạy ứng dụng
    SAGIN_NOMA_App;
end