# generated from colcon_powershell/shell/template/package.ps1.em

# function to append a value to a variable
# which uses colons as separators
# duplicates as well as leading separators are avoided
# first argument: the name of the result variable
# second argument: the value to be prepended
function colcon_append_unique_value {
  param (
    $_listname,
    $_value
  )

  # get values from variable
  if (Test-Path Env:$_listname) {
    $_values=(Get-Item env:$_listname).Value
  } else {
    $_values=""
  }
  $_duplicate=""
  # start with no values
  $_all_values=""
  # iterate over existing values in the variable
  if ($_values) {
    $_values.Split(";") | ForEach {
      # not an empty string
      if ($_) {
        # not a duplicate of _value
        if ($_ -eq $_value) {
          $_duplicate="1"
        }
        if ($_all_values) {
          $_all_values="${_all_values};$_"
        } else {
          $_all_values="$_"
        }
      }
    }
  }
  # append only non-duplicates
  if (!$_duplicate) {
    # avoid leading separator
    if ($_all_values) {
      $_all_values="${_all_values};${_value}"
    } else {
      $_all_values="${_value}"
    }
  }

  # export the updated variable
  Set-Item env:\$_listname -Value "$_all_values"
}

# function to prepend a value to a variable
# which uses colons as separators
# duplicates as well as trailing separators are avoided
# first argument: the name of the result variable
# second argument: the value to be prepended
function colcon_prepend_unique_value {
  param (
    $_listname,
    $_value
  )

  # get values from variable
  if (Test-Path Env:$_listname) {
    $_values=(Get-Item env:$_listname).Value
  } else {
    $_values=""
  }
  # start with the new value
  $_all_values="$_value"
  # iterate over existing values in the variable
  if ($_values) {
    $_values.Split(";") | ForEach {
      # not an empty string
      if ($_) {
        # not a duplicate of _value
        if ($_ -ne $_value) {
          # keep non-duplicate values
          $_all_values="${_all_values};$_"
        }
      }
    }
  }
  # export the updated variable
  Set-Item env:\$_listname -Value "$_all_values"
}

# function to source another script with conditional trace output
# first argument: the path of the script
# additional arguments: arguments to the script
function colcon_package_source_powershell_script {
  param (
    $_colcon_package_source_powershell_script
  )
  # source script with conditional trace output
  if (Test-Path $_colcon_package_source_powershell_script) {
    if ($env:COLCON_TRACE) {
      echo ". '$_colcon_package_source_powershell_script'"
    }
    . "$_colcon_package_source_powershell_script"
  } else {
    Write-Error "not found: '$_colcon_package_source_powershell_script'"
  }
}


# a powershell script is able to determine its own path
# the prefix is two levels up from the package specific share directory
$env:COLCON_CURRENT_PREFIX=(Get-Item $PSCommandPath).Directory.Parent.Parent.FullName

colcon_package_source_powershell_script "$env:COLCON_CURRENT_PREFIX\share/chrono_ros_interfaces/hook/cmake_prefix_path.ps1"
colcon_package_source_powershell_script "$env:COLCON_CURRENT_PREFIX\share/chrono_ros_interfaces/hook/ld_library_path_lib.ps1"
colcon_package_source_powershell_script "$env:COLCON_CURRENT_PREFIX\share/chrono_ros_interfaces/hook/pythonpath.ps1"
colcon_package_source_powershell_script "$env:COLCON_CURRENT_PREFIX\share/chrono_ros_interfaces/local_setup.ps1"

Remove-Item Env:\COLCON_CURRENT_PREFIX
