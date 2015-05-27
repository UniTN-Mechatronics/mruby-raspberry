#!/usr/bin/env ruby

if __FILE__ == $PROGRAM_NAME
  require 'fileutils'
  FileUtils.mkdir_p 'tmp'
  unless File.exists?('tmp/mruby')
    system 'git clone --depth 1 https://github.com/mruby/mruby.git tmp/mruby'
  end
  exit system(%Q[cd tmp/mruby; MRUBY_CONFIG=#{File.expand_path __FILE__} ./minirake #{ARGV.join(' ')}])
end

MRuby::Build.new do |conf|
  toolchain :gcc
  conf.gembox 'default'
  conf.gem File.dirname(__FILE__)
  conf.gem :github => 'iij/mruby-errno', :branch => 'master'
  conf.gem :github => 'iij/mruby-pack', :branch => 'master'
  conf.cc.defines = %w(ENABLE_READLINE)
end