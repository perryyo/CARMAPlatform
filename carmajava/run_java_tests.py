#!/usr/bin/python

from subprocess import call
from shutil import copytree, rmtree
from os import environ, chdir, path, listdir
import xml.etree.ElementTree as ET

start_folder = environ["PWD"] # Make sure to return to this folder at the end
devel_prefix_path = environ["CMAKE_PREFIX_PATH"].partition("/devel:")
project_root = devel_prefix_path[0] # THIS MAY ONLY BE BASED ON CONVENTION, PROBABLY BRITTLE

# Supporting alternate checkout locations
if path.exists(project_root + "/src/carmajava"):
    carmajava_path = project_root + "/src/carmajava"
elif path.exists(project_root + "/src/CarmaPlatform/carmajava"):
    carmajava_path = project_root + "/src/CarmaPlatform/carmajava"
else:
    carmajava_path = project_root + "/src/host/home/ubuntu/CarmaPlatform/carmajava"

print ">>> Moving into project directory..."
chdir(carmajava_path)
print ">>> Gradling tests..."

test_result_code = call([carmajava_path + "/gradlew", "test"])

if test_result_code == 1:
    print ">>> Gradle testing successful."
else:
    print ">>> Gradle testing FAILED! Exit code: " + str(test_result_code)

print ">>> Copying test results..."
test_destination_folder = project_root + "/build/test_results/carma/"
test_result_folder = carmajava_path + "/build/test-results"
if not path.exists(test_result_folder):
    print ">>> No test results found..."
    print ">>> Returning to build folder..."
    chdir(start_folder)
    print ">>> Exiting..."
    exit(1)

if path.exists(test_destination_folder):
    rmtree(test_destination_folder)
copytree(test_result_folder, test_destination_folder)
rmtree(test_result_folder)

print ">>> Create aggregate test result report..."
aggregate_test_root = ET.Element("testsuite")
aggregate_test_root.set("name", "aggregate-java-tests")

num_tests = 0
num_errors = 0
num_failures = 0
num_skipped = 0
total_time = 0
timestamp = None
for filename in listdir(test_destination_folder):
    if (filename.endswith(".xml")):
        try:
            tree = ET.parse(test_destination_folder + filename)
            test_suite_elem = tree.getroot()
            if not timestamp:
                timestamp = test_suite_elem.get("timestamp")
            num_tests += int(test_suite_elem.get("tests"))
            num_errors += int(test_suite_elem.get("errors"))
            num_failures += int(test_suite_elem.get("failures"))
            num_skipped += int(test_suite_elem.get("skipped"))
            total_time += float(test_suite_elem.get("time"))
            for case in test_suite_elem.findall("testcase"):
                aggregate_test_root.append(case)
        except ET.ParseError:
            ValueError(filename)

aggregate_test_root.set("tests", str(num_tests))
aggregate_test_root.set("errors", str(num_errors))
aggregate_test_root.set("failures", str(num_failures))
aggregate_test_root.set("skipped", str(num_skipped))
aggregate_test_root.set("time", str(total_time))
if timestamp:
    aggregate_test_root.set("timestamp", timestamp)

aggregate_test_root.append(ET.Element("properties"))
aggregate_test_root.append(ET.Element("system-out"))
aggregate_test_root.append(ET.Element("system-err"))

expected_test_result_file = test_destination_folder + "nosetests-run_java_tests.py.xml"
ET.ElementTree(aggregate_test_root).write(expected_test_result_file)

# Restore preconditions and exit
print ">>> Returning to build folder..."
chdir(start_folder)
exit(test_result_code)
