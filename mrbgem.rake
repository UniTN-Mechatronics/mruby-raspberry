MRuby::Gem::Specification.new('mruby-raspberry') do |spec|
  # Note that it needs WiringPI libraries (http://wiringpi.com)
  spec.license = 'MIT'
  spec.author  = 'Paolo Bosetti and Matteo Ragni, University of Trento'
  spec.summary = 'Raspberry library for IO access'
  spec.version = 0.1
  spec.description = spec.summary
  spec.homepage = "Not yet defined"
  
  spec.cc.command = 'gcc' # clang does not work!
  spec.cc.flags << %w||
  spec.cc.include_paths << "/usr/local/include"
  
  spec.linker.library_paths << "/usr/local/lib"
  spec.linker.libraries << %w[wiringPi]
end
