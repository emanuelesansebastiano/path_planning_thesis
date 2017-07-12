function [ path_file_values ] = funcReadSaveFilePath(file_name, save_bool)
%------------------------------------------------------------------------%
%funcReadSaveFilePath - convert a txt file into a cell matrix
%
% Author, date:
%   - Emanuele Sansebastiano, June 2017
%........................................................................%
%
% Input data:
%   - File name without the extension
%      eg: 'file2read' | 'file2read.txt' is wrong
%   - decisional value to save of not what has been read from the file
%      any positive value will trig the save function
%
% Output:
%   - values contained in a cell matrix
%   - file .mat saved in the folder where this function is if(save > 0)
%
% File description
%   - The txt file must contain the comments between '%' 
%   - Every line must start with the char 'b' or 'r' or 'l'
%   - Every not commented line must have this structure: 
%       %c \t %s \t %d \t %d \t %f \t %f \t %d 
%
%------------------------------------------------------------------------%

%% Common variables
comment_char = {'%'};
left_char = {'l'};
right_char = {'r'};
both_char = {'b'};
line = 0; comment_lines = 0;

file_type = '.txt';
file_directory = strcat(file_name, file_type);

%% Main program
fileID = fopen(file_directory,'r');
while(~feof(fileID))
    line = line +1;
    clear textscanned; textscanned = textscan(fileID,'%c',1);
    if (cellfun(@strcmp, comment_char, textscanned))
        comment_lines = comment_lines +1;
        clear textscanned; textscanned = textscan(fileID,'%c',1);
        while(~cellfun(@strcmp, comment_char, textscanned))
            clear textscanned; textscanned = textscan(fileID,'%c',1);
        end
    elseif (cellfun(@strcmp, left_char, textscanned) || cellfun(@strcmp, right_char, textscanned) || cellfun(@strcmp, both_char, textscanned))
        %both, right or left arm
        clear textscanned; textscanned = textscan(fileID,'%s%d%d%d%d',1);
        textscanned{1,end} = textscanned{1,end} +1;
        temp_size = size(textscanned,2);
        for i = 1 : textscanned{1,temp_size}*2
            clear textscanned; textscanned = textscan(fileID,'%f%f%f%f%f%f%f',1);
        end
    else
        %line error
        fprintf('Error: The file is not as it is expected to be. Check the line %d.\n', line);
        break;
    end
end
fclose(fileID);

%% Main program repeated to save the cell file correctly
%values stored in the read file
path_file_values = cell(line - comment_lines, size(textscanned,2) +1);
line = 0;

fileID = fopen(file_directory,'r');
while(~feof(fileID))
    clear textscanned; textscanned = textscan(fileID,'%c',1);
    if (cellfun(@strcmp, comment_char, textscanned))
        clear textscanned; textscanned = textscan(fileID,'%c',1);
        while(~cellfun(@strcmp, comment_char, textscanned))
            clear textscanned; textscanned = textscan(fileID,'%c',1);
        end
    else
        line = line +1;
        path_file_values(line, 1) = textscanned;
        %both, right or left arm
        clear textscanned; textscanned = textscan(fileID,'%s%d%d%d%d',1);
        temp_size = size(textscanned,2);
        path_file_values(line, 1 +1 : 1 +temp_size);
        for i = 1 : textscanned{1,temp_size}
            clear textscanned; textscanned = textscan(fileID,'%f%f%f%f%f%f%f',1);
            path_file_values(line, 1 +1 +temp_size + (i-1)*size(textscanned,2) : 1 +temp_size + (i)*size(textscanned,2));
        end
    end
end
fclose(fileID);

clear ans both_char left_char right_char comment_char comment_lines...
    file_directory fileID line textscanned fil

%save the data stored in the file
if(save_bool)
    save(file_name);
end

end


