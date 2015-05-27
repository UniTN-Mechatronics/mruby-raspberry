#*************************************************************************#
#                                                                         #
# raspberry.rb - mruby gem provoding access to Raspberry Pi IOs           #
# Copyright (C) 2015 Paolo Bosetti and Matteo Ragni,                      #
# paolo[dot]bosetti[at]unitn.it and matteo[dot]ragni[at]unitn.it          #
# Department of Industrial Engineering, University of Trento              #
#                                                                         #
# This library is free software.  You can redistribute it and/or          #
# modify it under the terms of the GNU GENERAL PUBLIC LICENSE 2.0.        #
#                                                                         #
# This library is distributed in the hope that it will be useful,         #
# but WITHOUT ANY WARRANTY; without even the implied warranty of          #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           #
# Artistic License 2.0 for more details.                                  #
#                                                                         #
# See the file LICENSE                                                    #
#                                                                         #
#*************************************************************************#

module Raspberry
  def self.included(base)
    self::setup
  end

  class Serial
    BAUD_RATES = [110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
    
    def valid_rate?(rate)
      BAUD_RATES.include? @baud
    end
    
    def open?
      @device && @device >= 0
    end
    def closed?; ! self.open?; end
    
    def printf(str, *data)
      self.puts(sprintf(str, *data))
    end
  end
  
  class I2C
    attr_accessor :delay
    
    def read_chr
      self.read.chr
    end
    
    def read(n=nil)
      if n && n > 0 then
        raise "ArgumentError", "Only Fixnum as argument" unless n.kind_of? Fixnum
        ary = []
        n.times {|i| ary << self._read; Raspberry::Timing.delay_micro(@delay || 5000) }
        return ary
      else
        return self._read
      end
    end
    
    def read_ary(n=1)
      self._read_ary(n)
    end
    
    def read_unpack(n, map=nil)
      map = 'C' * n unless map
      self._read_str(n).unpack(map)
    end
    
    def write(s)
      case s
      when Array
        s.each {|c| self.write c; Raspberry::Timing.delay_micro(@delay || 5000) }
      when String
        if s.length == 0 then
          self._write(s.bytes[0])
        else
          self.write(s.bytes)
        end
      when Fixnum
        self._write(s)
      else
        raise ArgumentError, "Only Strings, Fixnums, or Array of Fixnums"
      end
    end
  end
end
