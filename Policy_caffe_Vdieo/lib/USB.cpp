//
// Created by Administrator on 2019/7/31.
//

#include<stdio.h>
//#include<libusb.h>
#ifndef _WIN32

#include<libusb-1.0/libusb.h>
#include<sys/types.h>
#include<errno.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<stdlib.h>
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>

#endif

#include <glog/logging.h>
#include <boost/format.hpp>
#include "USB.hpp"
#include <vector>
#include <tuple>
#include <set>
#include <boost/filesystem.hpp>


struct usb_t {
    int b;
    int d;
    uint16_t vendor;
    uint16_t product;

    usb_t(int b_, int d_, uint16_t v, uint16_t p) : b(b_), d(d_), vendor(v), product(p) {

    }

    usb_t() {

    }
};


using namespace terjin;
namespace fs = boost::filesystem;

class USBImpl : public terjin::USB {
public:
    USBImpl() {
#ifndef _WIN32
        libusb_context *_ctx;

        libusb_device **_list;

        struct libusb_device_descriptor _desc{};

        _err = libusb_init(&_ctx);
        if (_err)
            throw std::runtime_error((boost::format("Open USB Failed: %d") % _err).str());

        libusb_device_handle *handle = nullptr;
        int config = 0;

        ssize_t num_devs, i;

        num_devs = libusb_get_device_list(_ctx, &_list);
        LOG(INFO) << "Num of Devs: " << num_devs;

        for (i = 0; i < num_devs; ++i) {
            LOG(INFO) << "Open Dev: " << i;
            libusb_device *dev = _list[i];
            _err = libusb_open(dev, &handle);
            if (_err) {
                LOG(ERROR) << "open dev " << i << " err: " << _err;
                continue;
            }
            _err = libusb_get_configuration(handle, &config);
            if (_err) {
                LOG(ERROR) << "get configuration for " << i << " err: " << _err;
                libusb_close(handle);
                continue;
            }
            uint8_t bnum = libusb_get_bus_number(dev);
            uint8_t dnum = libusb_get_device_address(dev);
            _err = libusb_get_device_descriptor(dev, &_desc);
            if (_err) {
                LOG(INFO) << "get device descriptor for " << i << " err: " << _err;
                libusb_close(handle);
                continue;
            }
            LOG(INFO) << "device: " << std::hex << _desc.idVendor << ":" << _desc.idProduct << std::dec << std::endl;
            _devs.emplace_back(bnum, dnum, _desc.idVendor, _desc.idProduct);
            libusb_close(handle);
        }

        libusb_free_device_list(_list, 1);
        libusb_exit(_ctx);

#endif
    }

    ~USBImpl() {
#ifndef _WIN32
#endif

    }

    bool CheckIfExists(uint16_t idVendor, uint16_t idProduct) override {
        LOG(INFO) << "target device: " << std::hex << idVendor << ":" << idProduct << std::dec << std::endl;
        for (auto &_dev : _devs) {
            if (idVendor == _dev.vendor && idProduct == _dev.product) {
                return true;
            }
        }
        return false;
    }

    int reset_usb(uint16_t idVendor, uint16_t idProduct) override {
#ifndef _WIN32

        bool found = false;
        usb_t u{};
        LOG(INFO) << "device: " << std::hex << idVendor << ":" << idProduct << std::dec << std::endl;
        for (auto &_dev : _devs) {
            if (idVendor == _dev.vendor && idProduct == _dev.product) {
                u = _dev;
                found = true;
            }
        }

        if (!found) {
            LOG(INFO) << std::hex << idVendor << ":" << idProduct << " not found";
            return 0;
        }

        std::string filename = (boost::format("/dev/bus/usb/%03d/%03d") % u.b % u.d).str();
        LOG(INFO) << "reset " << filename;
        int fd;
        fd = open(filename.c_str(), O_WRONLY);
        if (fd < 0) {
            return fd;
        }
        LOG(INFO) << "reset result: " << ioctl(fd, USBDEVFS_RESET, 0);
        close(fd);
#endif
        return 0;
    }

    int reset_usb() override {
        /* Okay, first, we need to discover what the path is to the ehci and
         * xhci device files. */
        std::set<fs::path> path_list;
        path_list.insert("/sys/bus/pci/drivers/xhci-pci/");
        path_list.insert("/sys/bus/pci/drivers/ehci-pci/");
        path_list.insert("/sys/bus/pci/drivers/xhci_hcd/");
        path_list.insert("/sys/bus/pci/drivers/ehci_hcd/");

        LOG(WARNING) << "TODO Rebind xhci_hcd is disabled";
        return 0;
        /* Check each of the possible paths above to find which ones this system
         * uses. */
        for (std::set<fs::path>::iterator found = path_list.begin();
             found != path_list.end(); ++found) {

            if (fs::exists(*found)) {

                fs::path devpath = *found;

                std::set<fs::path> globbed;

                /* Now, glob all of the files in the directory. */
                fs::directory_iterator end_itr;
                for (fs::directory_iterator itr(devpath); itr != end_itr; ++itr) {
                    globbed.insert((*itr).path());
                }

                /* Check each file path string to see if it is a device file. */
                for (std::set<fs::path>::iterator it = globbed.begin();
                     it != globbed.end(); ++it) {

                    std::string file = fs::path((*it).filename()).string();

                    if (file.length() < 5)
                        continue;

                    if (file.compare(0, 5, "0000:") == 0) {
                        /* Un-bind the device. */
                        std::fstream unbind((devpath.string() + "unbind").c_str(),
                                            std::fstream::out);
                        unbind << file;
                        unbind.close();

                        /* Re-bind the device. */
                        LOG(INFO) << "Re-binding: " << file << " in "
                                  << devpath.string();
                        std::fstream bind((devpath.string() + "bind").c_str(),
                                          std::fstream::out);
                        bind << file;
                        bind.close();
                    }
                }
            }
        }

        return 0;
    }

private:
    int _err;

    std::vector<usb_t> _devs;
};

USB::sptr USB::make() {
    return terjin::USB::sptr(new USBImpl());
}
