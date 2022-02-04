# Allied Vision GigE camera by Volley LLC

#### Repository for storing Allied Vision's Vimba SDK related software

The Gigabit Ethernet Vision standard defines the process for the host machine  
to discover, control, and acquire images from connected GigE Vision cameras.  
In order to discover Allied Vision recording device connected to the GigE bus  
Vimba transport layers must be installed and OS restarted. Refer to installation  
guide at the bottom of this document.

For `vision` daemon to manage connected camera's discovery, initialization and  
image acquisition it utilizes Vimba-SDK and its C++ API. Each camera can be  
configured within our software by loading the `settings.xml` file. These `*.xml`  
profiles can be generated with the `VimbaViewer` and to update the Camera's  
firmware one should use `FirmwareUpdater` tool that are part of the Vimba-SDK.  
The copy of the full SDK used to compile the `vision` code can be found on our  
google drive: [Google-drive-download].

For further reading on the Vimba C++ API refer to its manual located in the  
`VimbaCPP/Documentation` [directory](VimbaCPP/Documentation/Vimba_CPP_Manual.pdf).


### Official download page and Linux installation instruction:
  - [Vimba-download]
  - [Vimba-installation-guide]

[Google-drive-download]:https://drive.google.com/drive/folders/1dlHUktkD-h2mUBXvPc04Vcxiup95a56u
[Vimba-download]:https://www.alliedvision.com/en/products/software.html#c6444
[Vimba-installation-guide]:https://cdn.alliedvision.com/fileadmin/content/documents/products/software/software/Vimba/appnote/Vimba_installation_under_Linux.pdf

