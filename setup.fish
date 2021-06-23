#!/usr/bin/env fish

set script_dir (realpath (dirname (status --current-filename)))

set -l options "c/clean"
argparse $options -- $argv

set -l snake_case_package_name (basename $script_dir)
set -l PascalCasePackageName (echo "$snake_case_package_name" | sed -E 's/^(.)/\U\1/g' | sed -E 's/_(.)/\U\1/g')
set -l UPPER_SNAKE_CASE_PACKAGE_NAME (basename $script_dir | sed -E 's/(.*)/\U\1/g')


cd $script_dir
sd "GIT_CONFIG_EMAIL@example.com" (git config --global user.email) $script_dir/package.xml
sd "GIT_CONFIG_NAME" (git config --global user.name) $script_dir/package.xml

set -l target_files (fd -t f --exclude "*.fish" --exclude "README.md")
sd "package_name" $snake_case_package_name $target_files
sd "PackageName" $PascalCasePackageName $target_files
sd "PACKAGE_NAME" $UPPER_SNAKE_CASE_PACKAGE_NAME $target_files

rename -s "package_name" "$snake_case_package_name" (fd -t d)
rename -s "package_name" "$snake_case_package_name" (fd -t f --exclude "*.fish")

if set -q _flag_clean
    mv $script_dir/PACKAGE_README.md $script_dir/README.md
    rm -rf $script_dir/.git
    rm $script_dir/setup.fish
else
    echo -n "Please run the following commands manually to clean up the package.
  mv $script_dir/PACKAGE_README.md $script_dir/README.md
  rm $script_dir/setup.fish
  rm -rf $script_dir/.git # When you add this to an existing repository
"
end
