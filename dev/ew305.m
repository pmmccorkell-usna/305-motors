% # Patrick McCorkell
% # February 2022
% # US Naval Academy
% # Robotics and Control TSD
% #

classdef ew305 < matlab.mixin.SetGet
	properties(GetAccess='public', SetAccess='public')
        % serial management:
        com_port
		com_port_available
		serial_device
        buffer

        % parameters to microcontroller:
        target
		Kp
		Ki
		Kd
		runtime
		bias
        rate

        % data from microcontroller:
		data
	end

	methods(Access='public')
		% absolute necessary args in: COM port as an integer.
        function obj=ew305(varargin)
            % Some initial serial bookkeeping.
			obj.com_port = string("COM"+string(varargin{1}));
			obj.com_port_available = 0;

            % Default parameters.
            obj.target = 0;     % rad/s
			obj.Kp = 9.42e-8;
			obj.Ki = 1.256e-6;
			obj.Kd = 3.14e-11;
			obj.runtime = 1.0;  % s
			obj.bias = 0;       % x / 511
            obj.rate = 500;     % Hz

            % If the serial port is available, then open a connection to it.
            %   Otherwise, throw an error.
			obj.is_port_available();
			if (obj.com_port_available)
				obj.init_serial();
			else
				fprintf("Serial Port " + obj.com_port + " not available.\nClear port and try again manually\n");
			end
		end % end function

        % Things to do when class is destroyed.
		function delete(obj)
			obj.deinit_serial();
		end % end function

        % Destroy the serial device.
		function deinit_serial(obj)
			obj.serial_device = 0
	 	end % end function

        % Reset RP2040 over serial.
        function reset_rp2040(obj)
            % ASCII equivalents of keystrokes
            ctrl_c = 0x3;
            ctrl_d = 0x4;

            % First send a keyboard interrupt.
            obj.serial_device.write(ctrl_c,'uint8');
            pause(0.1);

            % Then send a software reset instruction.
            obj.serial_device.write(ctrl_d,'uint8');

            % Flush the serial interface.
            %   Necessary to wipe out Adafruit Circuitpython's opening messages, 
            %   which cannot be parsed by json.
            pause(0.5);
            obj.serial_device.flush();
        end % end function

        % Scrape serial ports to see if the supplied one is available.
		function is_port_available(obj)
			obj.com_port_available = 0;

            % Get all the serial ports that are open.
			available_ports = serialportlist("available");

            % Iterate through open ports looking for a match to ours.
			for i=1:length(available_ports)
				if (available_ports(i) == obj.com_port)
					obj.com_port_available = 1; % Oh happy days.
				end
			end
		end % end function

        % Initialize the serial port.
		function init_serial(obj)
            % Open the port.
			obj.serial_device = serialport(obj.com_port,115200);

            % Flush the port with extreme prejudice to get a clean slate.
            pause(0.5);
			while(obj.serial_device.NumBytesAvailable)
				obj.buffer = readline(serial.device);
				fprintf(obj.buffer);
			end
			obj.serial_device.flush();
		end % end function

		% obj.send_commands(target,runtime,Kp,Ki,Kd, motor bias, sample rate)
		%   At least 'target' must be sent as an argument.
		%   Optional arguments: runtime, Kp, Ki, Kd, motor bias, sample rate
		function send_commands(obj,varargin)

            % Some book keeping.
            % Assign the parameters as necessary.
			obj.target = varargin{1};
			if nargin > 2
				obj.runtime = varargin{2};  % sample time
			end
			if nargin >3
				obj.Kp = varargin{3};       % Kp
			end
			if nargin > 4
				obj.Ki = varargin{4};       % Ki
			end
            if nargin > 5
				obj.Kd = varargin{5};       % Kd
            end
            if nargin > 6
				obj.bias = varargin{6};     % motor bias
            end
            if nargin > 7
                obj.rate = varargin{7};     % sample rate; experimental. Results may vary.
            end

            % Reset the RP2040 after each experiment.
            %   MemAlloc issues may arise if not restarted.
            obj.reset_rp2040();

            % Form the json string and fire it off into the ether.
			commands = struct('target',obj.target,'Kp',obj.Kp,'Ki',obj.Ki,'Kd',obj.Kd,'time_limit',obj.runtime,'bias',obj.bias, 'rate',obj.rate);
			obj.serial_device.writeline(jsonencode(commands));

            % Due to post processing, hang out until the experiment is designed to finish.
            pause(obj.runtime + 1);
			obj.read_serial();
            obj.graph_v()
        end % end function

        % Filter out Debugging messages from the RP2040.
        function returnval = check_serial_LOG(obj)
            % All Debugging messages start with this:
            log_check = 'LOG:';
            % Turn str into char array and slice.
            buffer_char = char(obj.buffer);
            buffer_char = buffer_char(1:4);
            % str compare the buffer vs. the debugging prefix.
            returnval = strcmp(log_check,buffer_char);
        end % end function

        % Read the serial data and parse from json to struct.
        function read_serial(obj)
            % Create an empty dummy struct to throw data into later.
            obj.data = struct('position',0,'time',0);
            % Create an empty buffer.
            obj.buffer = '';
            % Wait for the electrons to settle down and the stars to align.
			pause(0.1);


            % While there are bytes available in the serial device,
                % continue churning and gathering the data into a struct
                % array.
   			i=0; % Counter for data struct array.
			while(obj.serial_device.NumBytesAvailable)
                % Egyptians discovered zero some 3500 years ago, Olmecs 2500 years ago. Hindus, Chinese, and Arabs formalized zero in Mathematics during the 7th and 8th centuries.
                %   And here, in 2022, MathWorks still believes the numbering system starts at one.
                i = i+1;
				obj.buffer = readline(obj.serial_device);

                % Filter out debugging messages from the microcontroller.
                %   They cannot be parsed as json into a struct.
                if (obj.check_serial_LOG())
                    disp(obj.buffer);

                % If not debugging messages, proceed with caution.
                else
                    % 'try except', because you never know what's really coming over serial.
                    try
                        % Decode json into a single struct.
				        json_data = jsondecode(obj.buffer);
                        % Add that struct to the end of the data struct array.
				        obj.data(i) = json_data;

                    % If we can't parse the data as json, throw errors and display the offending information.
                    catch
				        warning("Data not in proper json format");
                        disp("ERROR: ");
                        disp(obj.buffer);
                        i = i-1;    % decrement the counter to prevent a bad entry in the data struct array on next pass.
                    end % end try/except
                end % end if/else

                % Make sure the serial buffer is truly zeroed out, and stays 0.
                %   Sometimes after clearing the last line in serial buffer, 
                %   another line is still coming from the microcontroller.
                if (obj.serial_device.NumBytesAvailable==0)
                    pause(0.1);
                end

			end % end while
            fprintf("data samples: %d\n", length(obj.data));
		end % end function

        function graph_v(obj)
            i = 0;
            l = length(obj.data);
            v = zeros(l,1);
            dt = zeros(l,1);
            dx = zeros(l,1);
            while (i < (l - 1))
                dx(l-i) = (obj.data(end-i).position - obj.data(end-i-1).position);
                dt(l-i) = (obj.data(end-i).time - obj.data(end-i-1).time);
                v(l - i) = dx(l-i) / dt(l-i);
                i = i + 1;
            end
            figure;
            plot([obj.data.time],v);
            hold on
            xlabel('time (s)');
            ylabel('angular velocity (rad/s)');
            hold off
        end

    end % end methods
end % end class