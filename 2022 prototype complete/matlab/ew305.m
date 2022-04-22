% # Patrick McCorkell
% # November 2021
% # US Naval Academy
% # Robotics and Control TSD
% #

% ex:
% a = ew305(15)                     where 15 port#... COM15
% a.send_commands(target, runtime, Kp, Ki, Kd, dc bias)      ... only target is required
% a.data                            to see the data

classdef ew305 < matlab.mixin.SetGet
	properties(GetAccess='public', SetAccess='public')
		com_port
		com_port_available
		serial_device
		target
		Kp
		Ki
		Kd
		runtime
		bias
        rate
		data
        buffer
	end

	methods(Access='public')
		% absolute necessary args in: COM port, target location
		% optional args in: Kp, Ki, Kd, runtime, min_drive
		function obj=ew305(varargin)
			obj.com_port = string("COM"+string(varargin{1}));

			% obj.init_serial();
			obj.com_port_available = 0;

            % Default commands
            % obj.target = 10;
			obj.Kp = 9.42e-8;
			obj.Ki = 1.256e-6;
			obj.Kd = 3.14e-11;
			obj.runtime = 1.0;
			obj.bias = 0;
            obj.rate = 500;

			obj.is_port_available();
			if (obj.com_port_available)
				obj.init_serial();
			else
				fprintf("Serial Port " + obj.com_port + " not available.\nClear port and try again manually\n");
			end

		end % end function

		function delete(obj)
			obj.deinit_serial();
		end % end function

		function deinit_serial(obj)
			obj.serial_device = 0
	 	end % end function

        function reset_rp2040(obj)
            ctrl_c = 0x3;
            ctrl_d = 0x4;
            obj.serial_device.write(ctrl_c,'uint8')
            pause(0.1);
            obj.serial_device.write(ctrl_d,'uint8')
            pause(0.1)
            obj.serial_device.flush();
        end % end function

		function is_port_available(obj)
			obj.com_port_available = 0;
			available_ports = serialportlist("available");
			for i=1:length(available_ports)
				if (available_ports(i) == obj.com_port)
					obj.com_port_available = 1;
				end
			end
		end % end function

		function init_serial(obj)
			obj.serial_device = serialport(obj.com_port,115200);
			pause(0.5);
			while(obj.serial_device.NumBytesAvailable)
				obj.buffer = readline(serial.device);
				fprintf(obj.buffer);
			end
			obj.serial_device.flush();
		end % end function

		% obj.send_commands(target,runtime,Kp,Ki,Kd,min_drive)
		% At least 'target' must be sent as an argument.
		% Optional arguments: runtime, Kp, Ki, Kd, min_drive
		function send_commands(obj,varargin)
			obj.target = varargin{1};
			if nargin > 2
				obj.runtime = varargin{2};
			end

			if nargin >3
				obj.Kp = varargin{3};
			end

			if nargin > 4
				obj.Ki = varargin{4};
			end

			if nargin > 5
				obj.Kd = varargin{5};
			end

			if nargin > 6
				obj.bias = varargin{6};
            end

            if nargin > 7
                obj.rate = varargin{7};
            end

			obj.serial_device.flush();
			pause(0.1);
			commands = struct('target',obj.target,'Kp',obj.Kp,'Ki',obj.Ki,'Kd',obj.Kd,'time_limit',obj.runtime,'bias',obj.bias, 'rate',obj.rate);
			obj.serial_device.writeline(jsonencode(commands));
            pause(obj.runtime + 1)
			obj.read_serial();
			% disp(obj.data);
			% obj.plot_data();
            %last_error = obj.data(end).error;
            %disp(last_error);
            
		end % end function

        function returnval = check_serial_LOG(obj)
            log_check = 'LOG:';
            buffer_char = char(obj.buffer);
            buffer_char = buffer_char(1:4);
            returnval = strcmp(log_check,buffer_char);
        end
        
        function read_serial(obj)
            obj.data = struct('position',0,'time',0);
            obj.buffer = '';
			pause(0.1);

   			i=0;
			while(obj.serial_device.NumBytesAvailable)
				i = i+1;
				obj.buffer = readline(obj.serial_device);

                % Uncomment this line for faster live graphing.
                % Matlab can't keep up with the Microcontroller.
                %      If commented, graphing will start to lag significantly behind.
                %      If uncommented, data resolution will be lost.
        		% obj.serial_device.flush();
                %
                if (obj.check_serial_LOG())
                     disp(obj.buffer);
                else
			        try
				        json_data = jsondecode(obj.buffer);
				        obj.data(i) = json_data;
                    catch
				        warning("Data not in proper json format");
				        error = obj.buffer;
                        disp("ERROR: ");
                        disp(error);
                        i = i-1;
                    end
                end

                % Make sure it's really 0 and stays 0.
                if (obj.serial_device.NumBytesAvailable==0)
                    pause(0.1);
                end

			end % end while
            fprintf("data samples: %d\n", length(obj.data));
            obj.reset_rp2040();
		end % end function        


    end % end methods
end % end class