Source: @(Package.replace('-gz-','-gzgarden-'))
Section: misc
Priority: optional
Maintainer: @(Maintainer)
Build-Depends: debhelper (>= @(debhelper_version).0.0), @(', '.join(BuildDepends))
Homepage: @(Homepage)
Standards-Version: 3.9.2

Package: @(Package.replace('-gz-','-gzgarden-'))
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}, @(', '.join(Depends))
Conflicts: @(Package)
@[if Conflicts]Conflicts: @(', '.join(Conflicts))@\n@[end if]@
@[if Replaces]Replaces: @(', '.join(Replaces))@\n@[end if]@
Description: @(Description)
