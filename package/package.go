package respnow

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	rcore "github.com/jurgen-kluft/rcore/package"
	rwifi "github.com/jurgen-kluft/rwifi/package"
)

// respnow is a package for Arduino Esp32/Esp8266 projects.
const (
	repo_path = "github.com\\jurgen-kluft"
	repo_name = "respnow"
)

func GetPackage() *denv.Package {
	name := repo_name

	// dependencies
	corepkg := rcore.GetPackage()
	wifipkg := rwifi.GetPackage()

	// main package
	mainpkg := denv.NewPackage(repo_path, repo_name)
	mainpkg.AddPackage(corepkg)
	mainpkg.AddPackage(wifipkg)

	// main library
	mainlib := denv.SetupCppLibProject(mainpkg, name)
	mainlib.AddDependencies(corepkg.GetMainLib())
	mainlib.AddDependencies(wifipkg.GetMainLib())

	mainpkg.AddMainLib(mainlib)
	return mainpkg
}
