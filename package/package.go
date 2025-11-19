package rdno_espnow

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	rdno_core "github.com/jurgen-kluft/rdno_core/package"
)

// rdno_espnow is a package for Arduino Esp32/Esp8266 projects.
const (
	repo_path = "github.com\\jurgen-kluft"
	repo_name = "rdno_espnow"
)

func GetPackage() *denv.Package {
	name := repo_name

	// dependencies
	corepkg := rdno_core.GetPackage()

	// main package
	mainpkg := denv.NewPackage(repo_path, repo_name)
	mainpkg.AddPackage(corepkg)

	// esp32 library
	esp32lib := denv.SetupCppLibProjectForArduinoEsp32(mainpkg, name+"-esp32")
	esp32lib.AddDependencies(corepkg.GetMainLib())
	esp32lib.AddEnvironmentVariable("ESP32_SDK")
	esp32lib.AddInclude("{ESP32_SDK}", "libraries/WiFi", "src")
	esp32lib.AddInclude("{ESP32_SDK}", "libraries/Network", "src")
	esp32lib.SourceFilesFrom("{ESP32_SDK}", "libraries/WiFi", "src", ".cpp")
	esp32lib.SourceFilesFrom("{ESP32_SDK}", "libraries/Network", "src", ".cpp")

	// esp8266 library
	esp8266lib := denv.SetupCppLibProjectForArduinoEsp8266(mainpkg, name+"-esp8266")
	esp8266lib.AddDependencies(corepkg.GetMainLib())
	esp8266lib.AddEnvironmentVariable("ESP8266_SDK")
	esp8266lib.AddInclude("{ESP8266_SDK}", "libraries/ESP8266WiFi", "src")
	esp8266lib.SourceFilesFrom("{ESP8266_SDK}", "libraries/ESP8266WiFi", "src", ".cpp")

	// main library
	mainlib := denv.SetupCppLibProject(mainpkg, name)
	mainlib.AddDependencies(corepkg.GetMainLib())
	mainlib.AddDependency(esp32lib)
	mainlib.AddDependency(esp8266lib)

	mainpkg.AddMainLib(mainlib)
	mainpkg.AddMainLib(esp32lib)
	mainpkg.AddMainLib(esp8266lib)
	return mainpkg
}
