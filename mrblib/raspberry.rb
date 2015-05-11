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
  class Serial
    def open?
      @device && @device >= 0
    end
    def closed?; ! self.open?; end
    
    def printf(str, *data)
      self.puts(sprintf(str, *data))
    end
  end
end
